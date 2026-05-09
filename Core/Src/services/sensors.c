// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Sensor data acquisition and processing pipeline.
 *
 * @details
 * Reads raw IMU (MPU6500) and barometer (BMP280 or SPL06) data via BSP,
 * applies gyro bias calibration, accelerometer scale correction, 2nd-order
 * low-pass filtering, and barometer compensation math.
 *
 * Hardware dependencies:
 * - MPU6500 on SPI, configured for +/-2000 dps gyro, +/-16 g accel, 1 kHz ODR.
 * - Barometer on I2C1 (BMP280 or SPL06, auto-detected at init).
 * - EXTI4 (PA4) rising-edge interrupt for MPU DRDY.
 *
 * Concurrency:
 * - sensors_task runs at 1 kHz, wakes on DRDY semaphore.
 * - Filtered data published to single-element FreeRTOS queues (xQueueOverwrite).
 * - Gyro bias calibration runs inline in sensors_process_imu (lock-free).
 */

#include "services/sensors.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "bsp_sensors.h"
#include "i2c.h"
#include "platform/filter.h"

/* --------------------------------------------------------------------------
 * MPU6500 conversion constants — matched to BSP register configuration.
 * BSP sets GYRO_FS=2000dps, ACCEL_FS=16G, SMPLRT_DIV=0 (1000Hz sample).
 * -------------------------------------------------------------------------- */
#define SENSORS_DEG_PER_LSB ((2.0f * 2000.0f) / 65536.0f)
#define SENSORS_G_PER_LSB ((2.0f * 16.0f) / 65536.0f)

/* Sample rate matched to BSP MPU config (SMPLRT_DIV=0) */
#define IMU_SAMPLE_RATE 1000.0f

/* Low-pass filter cutoffs */
#define GYRO_LPF_CUTOFF_FREQ 80.0f
#define ACCEL_LPF_CUTOFF_FREQ 30.0f

/* Gyro bias calibration */
#define BIAS_SAMPLE_COUNT 1024
#define GYRO_VARIANCE_BASE 4000.0f

/* Accelerometer scale calibration */
#define ACC_SCALE_SAMPLES 200

/* I2C addresses for barometer (same on I2C1 bus) */
#define BMP280_I2C_ADDR 0x76u
#define SPL06_I2C_ADDR 0x76u

/* BMP280 register: calibration data starts at 0x88 (24 bytes) */
#define BMP280_CALIB_REG 0x88u
#define BMP280_CALIB_LEN 24u

/* SPL06 register: calibration coefficients start at 0x10 (18 bytes) */
#define SPL06_CALIB_REG 0x10u
#define SPL06_CALIB_LEN 18u

/* --------------------------------------------------------------------------
 * Per-device calibration data types (for compensation math only).
 * BMP280: Bosch datasheet section 3.11.3 compensation formulas.
 * SPL06:  Goertek datasheet sections 7.2–7.3.
 * -------------------------------------------------------------------------- */

/**
 * @brief  BMP280 factory calibration coefficients and runtime compensation state.
 *
 * @details
 * Holds trimming parameters read once from BMP280 NVM (register block 0x88)
 * and the intermediate variable `t_fine` required by Bosch compensation math.
 * `loaded` indicates whether calibration data has been successfully populated.
 */
typedef struct bmp280_cal {
  uint16_t dig_t1; /* Unsigned temperature calibration coefficient T1. */
  int16_t dig_t2;  /* Signed temperature calibration coefficient T2. */
  int16_t dig_t3;  /* Signed temperature calibration coefficient T3. */

  uint16_t
    dig_p1; /* Unsigned pressure calibration coefficient P1 (must be non-zero). */
  int16_t dig_p2; /* Signed pressure calibration coefficient P2. */
  int16_t dig_p3; /* Signed pressure calibration coefficient P3. */
  int16_t dig_p4; /* Signed pressure calibration coefficient P4. */
  int16_t dig_p5; /* Signed pressure calibration coefficient P5. */
  int16_t dig_p6; /* Signed pressure calibration coefficient P6. */
  int16_t dig_p7; /* Signed pressure calibration coefficient P7. */
  int16_t dig_p8; /* Signed pressure calibration coefficient P8. */
  int16_t dig_p9; /* Signed pressure calibration coefficient P9. */

  int32_t
    t_fine; /* Shared fine-resolution temperature term used by pressure compensation. */
  bool loaded; /* True after all calibration fields are read from the sensor. */
} bmp280_cal_t;

/**
 * @brief  SPL06 factory calibration coefficients and precomputed scale factors.
 *
 * @details
 * Stores coefficients parsed from SPL06 calibration registers and the pressure/
 * temperature scaling factors used to normalize raw ADC data before polynomial
 * compensation. `loaded` marks whether coefficient loading has completed.
 */
typedef struct spl06_cal {
  int16_t
    c0; /* Temperature polynomial coefficient c0 (half-scaled in formula). */
  int16_t c1; /* Temperature polynomial coefficient c1. */

  int32_t c00; /* Pressure polynomial constant term c00. */
  int32_t c10; /* Pressure polynomial linear pressure term c10. */

  int16_t c01; /* Pressure polynomial linear temperature term c01. */
  int16_t c11; /* Pressure polynomial cross term c11 (T * P). */
  int16_t c20; /* Pressure polynomial quadratic pressure term c20 (P^2). */
  int16_t c21; /* Pressure polynomial mixed term c21 (T * P^2). */
  int16_t c30; /* Pressure polynomial cubic pressure term c30 (P^3). */

  float kp; /* Pressure scale factor (selected from oversampling config table). */
  float
    kt; /* Temperature scale factor (selected from oversampling config table). */
  bool
    loaded; /* True after coefficient and scale-factor initialization completes. */
} spl06_cal_t;

/**
 * @brief  Gyroscope bias estimator state using a fixed-size ring buffer.
 *
 * @details
 * Buffers raw gyro samples for stationary detection and mean-bias extraction.
 * `buf_head` is the current write pointer into `buffer`; `is_buffer_filled`
 * indicates one full collection window is available for variance checking;
 * `is_bias_found` indicates convergence and validity of `bias`.
 */
typedef struct {
  axis3f_t bias;      /* Estimated per-axis gyro bias in raw LSB domain. */
  bool is_bias_found; /* True once variance check passes and bias is valid. */
  bool
    is_buffer_filled; /* True after at least one full buffer cycle is written. */
  axis3i16_t *buf_head; /* Next write position in the ring buffer. */
  axis3i16_t buffer
    [BIAS_SAMPLE_COUNT]; /* Raw gyro sample history for variance/mean computation. */
} bias_obj_t;

/* --------------------------------------------------------------------------
 * Private module state
 * -------------------------------------------------------------------------- */
static sensor_data_t sensors_data;
static axis3i16_t gyro_raw_val;
static axis3i16_t acc_raw_val;
static axis3f_t gyro_bias_val;
static bool gyro_bias_found_val;
static float acc_scale_val =
  1.0f; /* scale == 1.0 until accel calibration completes */
static bool sensors_is_init;

static lpf2p_data_t gyro_lpf[3];
static lpf2p_data_t acc_lpf[3];
static bsp_sensors_status_t dev_status;

static bias_obj_t gyro_bias_running;

static xQueueHandle accel_queue;
static xQueueHandle gyro_queue;
static xQueueHandle baro_queue;
static xSemaphoreHandle data_ready_sem;

static bmp280_cal_t bmp280_cal;
static spl06_cal_t spl06_cal;
static bsp_sensors_barometer_type_t baro_type;

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void sensors_process_imu(void);
static void sensors_process_baro(void);

/**
 * @brief  Apply the 2nd-order LPF to all three axes of @p in in-place.
 *
 * @param[in,out] lpf  Array of three lpf2p_data_t instances (X, Y, Z).
 * @param[in,out] in   Input/output axis data; filtered result overwrites the input.
 */
static void sensors_apply_lpf_3axis(lpf2p_data_t *lpf, axis3f_t *in);

/**
 * @brief  Reset a bias_obj_t to its initial state.
 *
 * @param[in,out] bias  Object to reset.
 */
static void sensors_bias_obj_init(bias_obj_t *bias);

/**
 * @brief  Write one raw gyro sample into the ring buffer.
 *
 * When the write pointer wraps around the end, is_buffer_filled is set,
 * enabling variance-based convergence checks on the next call to
 * sensors_find_bias_value.
 *
 * @param[in,out] bias  Ring-buffer state.
 * @param[in]     x     Raw gyro X (LSB).
 * @param[in]     y     Raw gyro Y (LSB).
 * @param[in]     z     Raw gyro Z (LSB).
 */
static void sensors_add_bias_sample(bias_obj_t *bias, int16_t x, int16_t y,
                                    int16_t z);

/**
 * @brief  Attempt to extract a valid gyro bias from the ring buffer.
 *
 * Returns false immediately if the buffer is not yet full.  On success,
 * stores the mean into bias->bias and sets is_bias_found.  If variance
 * exceeds GYRO_VARIANCE_BASE on any axis (motion detected), resets
 * is_buffer_filled so collection restarts from scratch.
 *
 * @param[in,out] bias  Ring-buffer state.
 * @return true  Bias converged and stored in bias->bias.
 * @return false Buffer not full, or motion detected (collection reset).
 */
static bool sensors_find_bias_value(bias_obj_t *bias);

/**
 * @brief  Feed one gyro sample into the running bias estimator.
 *
 * Adds the sample to the ring buffer and, once the buffer is full,
 * attempts convergence.  Copies the result to the module-level
 * gyro_bias_val / gyro_bias_found_val globals used by sensors_process_imu.
 *
 * @param[in] gx  Raw gyro X in board frame (LSB).
 * @param[in] gy  Raw gyro Y in board frame (LSB).
 * @param[in] gz  Raw gyro Z in board frame (LSB).
 */
static void sensors_calibrate_gyro(int16_t gx, int16_t gy, int16_t gz);

/**
 * @brief  One-shot accelerometer scale calibration.
 *
 * Accumulates ACC_SCALE_SAMPLES vector magnitudes.  A stationary quad should
 * read exactly 1.0 g; the resulting average corrects per-unit sensitivity
 * variation.  Does nothing after scale_found is set.
 *
 * @note   Only called after gyro bias has converged to avoid magnifying
 *         motion-contaminated early samples.
 *
 * @param[in] ax  Raw accel X in board frame (LSB).
 * @param[in] ay  Raw accel Y in board frame (LSB).
 * @param[in] az  Raw accel Z in board frame (LSB).
 */
static void sensors_calibrate_accel(int16_t ax, int16_t ay, int16_t az);

/**
 * @brief  Compute per-axis variance and mean over the full ring buffer.
 *
 * Uses integer accumulators (int64) to avoid float overflow over 1024 samples.
 * Variance is the unnormalized sum-of-squares minus mean-squares quantity --
 * sufficient for threshold comparison without dividing by N.
 *
 * @param[in]  bias  Ring buffer (must be fully filled before calling).
 * @param[out] var   Per-axis variance (raw LSB^2, unnormalized).
 * @param[out] mean  Per-axis mean (raw LSB).
 */
static void sensors_compute_variance_and_mean(const bias_obj_t *bias,
                                              axis3f_t *var, axis3f_t *mean);

/**
 * @brief  Read BMP280 factory calibration coefficients from the sensor.
 *
 * Reads 24 bytes starting at register 0x88 (Bosch datasheet Table 16).
 * Silently returns on I2C failure -- compensation math will produce
 * garbage output but will not fault.
 */
static void sensors_load_bmp280_calib(void);

/**
 * @brief  Read SPL06 factory calibration coefficients from the sensor.
 *
 * Reads 18 bytes from register 0x10 (Goertek datasheet section 7.2).
 * kp and kt are pre-converted to float from the fixed scale-factor table
 * to avoid repeated table lookups at runtime.
 *
 * NOTE: scale factor indices are hardcoded to OVR_P=64x (index 6) and
 * OVR_T=8x (index 3) to match the BSP register configuration.
 */
static void sensors_load_spl06_calib(void);

/**
 * @brief  Convert compensated BMP280 ADC temperature to degrees Celsius.
 *
 * Bosch datasheet rev 1.21, section 3.11.3 integer formula.
 * Side effect: updates bmp280_cal.t_fine, which is required by
 * bmp280_compensate_p before pressure can be computed.
 *
 * @param[in] adc_T  Raw 20-bit ADC temperature value.
 * @return Compensated temperature in units of 0.01 degC (divide by 100 for degC).
 */
static int32_t bmp280_compensate_t(int32_t adc_T);

/**
 * @brief  Convert compensated BMP280 ADC pressure to Pascals.
 *
 * Bosch datasheet rev 1.21, section 3.11.4 integer 64-bit formula.
 * bmp280_compensate_t MUST be called first to populate t_fine.
 *
 * @param[in] adc_P  Raw 20-bit ADC pressure value.
 * @return Pressure in Q24.8 fixed-point Pa (divide by 256 for Pa, by 25600 for hPa).
 * @retval 0  If division-by-zero guard triggers (dig_P1 == 0).
 */
static uint32_t bmp280_compensate_p(int32_t adc_P);

/**
 * @brief  Compute compensated temperature from a raw SPL06 reading.
 *
 * Goertek datasheet section 7.3, first-order polynomial using c0 and c1.
 *
 * @param[in] raw_temp  Raw 24-bit temperature ADC value from the sensor.
 * @return Compensated temperature in degC.
 */
static float spl06_compensate_temperature(int32_t raw_temp);

/**
 * @brief  Compute compensated pressure from raw SPL06 readings.
 *
 * Goertek datasheet section 7.3 polynomial.  Both raw values must come
 * from the same measurement cycle.
 *
 * @param[in] raw_press  Raw 24-bit pressure ADC value.
 * @param[in] raw_temp   Raw 24-bit temperature ADC value (used for cross-compensation).
 * @return Compensated pressure in Pa.
 */
static float spl06_compensate_pressure(int32_t raw_press, int32_t raw_temp);

/**
 * @brief  Convert absolute pressure to altitude above sea level.
 *
 * Uses the international barometric formula with a reference pressure of
 * 1015.7 hPa (local ground-level calibration constant).
 *
 * @param[in] pressure_hpa  Absolute pressure in hPa.
 * @return Altitude in meters.  Returns 0.0 for non-positive input.
 */
static float pressure_to_altitude(float pressure_hpa);

/**
 * @brief  Shift @p addr left by 1 to match STM32 HAL I2C address convention.
 *
 * HAL_I2C_Mem_Read expects the 7-bit address pre-shifted into bits [7:1].
 */
static uint16_t i2c_hal_addr(uint8_t addr);

/**
 * @brief  Read @p len bytes from register @p reg_addr on I2C device @p dev_addr.
 *
 * Blocking with a 50 ms timeout.  Used only for one-time calibration reads at
 * init -- not called from the 1 kHz task loop.
 *
 * @param[in]  dev_addr  7-bit I2C device address.
 * @param[in]  reg_addr  Register start address.
 * @param[out] buf       Destination buffer.
 * @param[in]  len       Number of bytes to read.
 * @return true  Read succeeded (HAL_OK).
 * @return false I2C error or timeout.
 */
static bool i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf,
                           uint16_t len);

/* ========================================================================
 * I2C helper (one-time calibration reads only — not for runtime data)
 * ======================================================================== */

static uint16_t i2c_hal_addr(uint8_t addr)
{
  return (uint16_t)(addr << 1);
}

static bool i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf,
                           uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, i2c_hal_addr(dev_addr), reg_addr,
                          I2C_MEMADD_SIZE_8BIT, buf, len, 50u) == HAL_OK;
}

/* ========================================================================
 * EXTI4 callback — data-ready interrupt handler
 *
 * PA4 / EXTI4 is configured by CubeMX as GPIO_Input with rising-edge
 * interrupt.  MPU6500 asserts its INT pin when a new measurement is
 * ready (1 kHz).  This ISR posts the semaphore that wakes sensors_task.
 * ======================================================================== */

void sensors_exti_callback(uint16_t pin)
{
  if (pin == GPIO_PIN_4) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(data_ready_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/* ========================================================================
 * Calibration — gyro bias
 *
 * Collects BIAS_SAMPLE_COUNT raw gyro readings in a ring buffer.
 * When full, computes per-axis variance.  If all axes are below
 * GYRO_VARIANCE_BASE the quad is stationary and the mean becomes
 * the bias.  If variance is too high the buffer is reset —
 * calibration restarts from scratch.
 * ======================================================================== */

static void sensors_bias_obj_init(bias_obj_t *bias)
{
  bias->is_buffer_filled = false;
  bias->is_bias_found = false;
  bias->buf_head = bias->buffer;
  memset(&bias->bias, 0, sizeof(bias->bias));
  memset(bias->buffer, 0, sizeof(bias->buffer));
}

static void sensors_add_bias_sample(bias_obj_t *bias, int16_t x, int16_t y,
                                    int16_t z)
{
  bias->buf_head->x = x;
  bias->buf_head->y = y;
  bias->buf_head->z = z;
  bias->buf_head++;

  if (bias->buf_head >= &bias->buffer[BIAS_SAMPLE_COUNT]) {
    bias->buf_head = bias->buffer;
    bias->is_buffer_filled = true;
  }
}

static void sensors_compute_variance_and_mean(const bias_obj_t *bias,
                                              axis3f_t *var, axis3f_t *mean)
{
  int64_t sum[3] = { 0 };
  int64_t sumsq[3] = { 0 };

  for (uint32_t i = 0; i < BIAS_SAMPLE_COUNT; i++) {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumsq[0] += (int64_t)bias->buffer[i].x * bias->buffer[i].x;
    sumsq[1] += (int64_t)bias->buffer[i].y * bias->buffer[i].y;
    sumsq[2] += (int64_t)bias->buffer[i].z * bias->buffer[i].z;
  }

  var->x = (float)(sumsq[0] - (sum[0] * sum[0]) / BIAS_SAMPLE_COUNT);
  var->y = (float)(sumsq[1] - (sum[1] * sum[1]) / BIAS_SAMPLE_COUNT);
  var->z = (float)(sumsq[2] - (sum[2] * sum[2]) / BIAS_SAMPLE_COUNT);

  mean->x = (float)sum[0] / BIAS_SAMPLE_COUNT;
  mean->y = (float)sum[1] / BIAS_SAMPLE_COUNT;
  mean->z = (float)sum[2] / BIAS_SAMPLE_COUNT;
}

static bool sensors_find_bias_value(bias_obj_t *bias)
{
  if (!bias->is_buffer_filled) {
    return false;
  }

  axis3f_t mean, variance;
  sensors_compute_variance_and_mean(bias, &variance, &mean);

  if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE &&
      variance.z < GYRO_VARIANCE_BASE) {
    bias->bias.x = mean.x;
    bias->bias.y = mean.y;
    bias->bias.z = mean.z;
    bias->is_bias_found = true;
    return true;
  }

  bias->is_buffer_filled = false;
  return false;
}

static void sensors_calibrate_gyro(int16_t gx, int16_t gy, int16_t gz)
{
  sensors_add_bias_sample(&gyro_bias_running, gx, gy, gz);

  if (!gyro_bias_running.is_bias_found) {
    sensors_find_bias_value(&gyro_bias_running);
  }

  gyro_bias_val.x = gyro_bias_running.bias.x;
  gyro_bias_val.y = gyro_bias_running.bias.y;
  gyro_bias_val.z = gyro_bias_running.bias.z;
  gyro_bias_found_val = gyro_bias_running.is_bias_found;
}

/* ========================================================================
 * Calibration — accelerometer scale
 *
 * Accumulates ACC_SCALE_SAMPLES magnitude readings.  A stationary quad
 * should read exactly 1.0 g.  The ratio corrects for per-unit sensitivity
 * variation.
 * ======================================================================== */

static void sensors_calibrate_accel(int16_t ax, int16_t ay, int16_t az)
{
  static float scale_sum = 0.0f;
  static uint32_t scale_count = 0;
  static bool scale_found = false;

  if (scale_found) {
    return;
  }

  float ax_g = (float)ax * SENSORS_G_PER_LSB;
  float ay_g = (float)ay * SENSORS_G_PER_LSB;
  float az_g = (float)az * SENSORS_G_PER_LSB;
  scale_sum += sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  scale_count++;

  if (scale_count >= ACC_SCALE_SAMPLES) {
    acc_scale_val = scale_sum / (float)ACC_SCALE_SAMPLES;
    scale_found = true;
  }
}

/* ========================================================================
 * Barometer calibration coefficient loading
 *
 * BMP280 and SPL06 require per-chip calibration coefficients stored in
 * their internal registers.  These are read once during init and held
 * in static structs for the compensation math.
 * ======================================================================== */

static void sensors_load_bmp280_calib(void)
{
  uint8_t raw[BMP280_CALIB_LEN];

  if (!i2c_read_bytes(BMP280_I2C_ADDR, BMP280_CALIB_REG, raw,
                      BMP280_CALIB_LEN)) {
    return;
  }

  bmp280_cal.dig_t1 = (uint16_t)(raw[0] | (raw[1] << 8));
  bmp280_cal.dig_t2 = (int16_t)(raw[2] | (raw[3] << 8));
  bmp280_cal.dig_t3 = (int16_t)(raw[4] | (raw[5] << 8));
  bmp280_cal.dig_p1 = (uint16_t)(raw[6] | (raw[7] << 8));
  bmp280_cal.dig_p2 = (int16_t)(raw[8] | (raw[9] << 8));
  bmp280_cal.dig_p3 = (int16_t)(raw[10] | (raw[11] << 8));
  bmp280_cal.dig_p4 = (int16_t)(raw[12] | (raw[13] << 8));
  bmp280_cal.dig_p5 = (int16_t)(raw[14] | (raw[15] << 8));
  bmp280_cal.dig_p6 = (int16_t)(raw[16] | (raw[17] << 8));
  bmp280_cal.dig_p7 = (int16_t)(raw[18] | (raw[19] << 8));
  bmp280_cal.dig_p8 = (int16_t)(raw[20] | (raw[21] << 8));
  bmp280_cal.dig_p9 = (int16_t)(raw[22] | (raw[23] << 8));
  bmp280_cal.loaded = true;
}

static const uint32_t spl06_scale_factors[8] = { 524288,  1572864, 3670016,
                                                 7864320, 253952,  516096,
                                                 1040384, 2088960 };

static void sensors_load_spl06_calib(void)
{
  uint8_t raw[SPL06_CALIB_LEN];

  if (!i2c_read_bytes(SPL06_I2C_ADDR, SPL06_CALIB_REG, raw, SPL06_CALIB_LEN)) {
    return;
  }

  spl06_cal.c0 = (int16_t)(((uint16_t)raw[0] << 4) | (raw[1] >> 4));
  spl06_cal.c1 = (int16_t)(((uint16_t)(raw[1] & 0x0F) << 8) | raw[2]);
  spl06_cal.c00 = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) |
                            (raw[5] >> 4));
  spl06_cal.c10 = (int32_t)(((uint32_t)(raw[5] & 0x0F) << 16) |
                            ((uint32_t)raw[6] << 8) | raw[7]);
  spl06_cal.c01 = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
  spl06_cal.c11 = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
  spl06_cal.c20 = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);
  spl06_cal.c21 = (int16_t)(((uint16_t)raw[14] << 8) | raw[15]);
  spl06_cal.c30 = (int16_t)(((uint16_t)raw[16] << 8) | raw[17]);

  spl06_cal.kp = (float)spl06_scale_factors[6];
  spl06_cal.kt = (float)spl06_scale_factors[3];
  spl06_cal.loaded = true;
}

/* ========================================================================
 * BMP280 compensation — Bosch datasheet rev 1.21, sections 3.11.3–3.11.4
 * ======================================================================== */

static int32_t bmp280_compensate_t(int32_t adc_T)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_t1 << 1))) *
                  ((int32_t)bmp280_cal.dig_t2)) >>
                 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_t1)) *
                    ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_t1))) >>
                   12) *
                  ((int32_t)bmp280_cal.dig_t3)) >>
                 14;
  bmp280_cal.t_fine = var1 + var2;
  return (bmp280_cal.t_fine * 5 + 128) >> 8;
}

static uint32_t bmp280_compensate_p(int32_t adc_P)
{
  int64_t var1 = (int64_t)bmp280_cal.t_fine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)bmp280_cal.dig_p6;
  var2 = var2 + ((var1 * (int64_t)bmp280_cal.dig_p5) << 17);
  var2 = var2 + (((int64_t)bmp280_cal.dig_p4) << 35);
  var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_p3) >> 8) +
         ((var1 * (int64_t)bmp280_cal.dig_p2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_p1) >> 33;
  if (var1 == 0) {
    return 0;
  }
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)bmp280_cal.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)bmp280_cal.dig_p8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_p7) << 4);
  return (uint32_t)p;
}

/* ========================================================================
 * SPL06 compensation — Goertek datasheet
 * ======================================================================== */

static float spl06_compensate_temperature(int32_t raw_temp)
{
  float fTsc = (float)raw_temp / spl06_cal.kt;
  return (float)spl06_cal.c0 * 0.5f + (float)spl06_cal.c1 * fTsc;
}

static float spl06_compensate_pressure(int32_t raw_press, int32_t raw_temp)
{
  float fTsc = (float)raw_temp / spl06_cal.kt;
  float fPsc = (float)raw_press / spl06_cal.kp;
  float qua2 = (float)spl06_cal.c10 +
               fPsc * ((float)spl06_cal.c20 + fPsc * (float)spl06_cal.c30);
  float qua3 =
    fTsc * fPsc * ((float)spl06_cal.c11 + fPsc * (float)spl06_cal.c21);
  return (float)spl06_cal.c00 + fPsc * qua2 + fTsc * (float)spl06_cal.c01 +
         qua3;
}

/* ========================================================================
 * Pressure → altitude conversion (international barometric formula)
 * Reference pressure: 1015.7 hPa
 * ======================================================================== */

static float pressure_to_altitude(float pressure_hpa)
{
  if (pressure_hpa <= 0.0f) {
    return 0.0f;
  }
  return 44330.0f * (powf(1015.7f / pressure_hpa, 0.190295f) - 1.0f);
}

/* ========================================================================
 * IMU data processing
 *
 * Reads raw IMU registers via BSP, applies coordinate mapping,
 * bias/scale correction, unit conversion, and low-pass filtering.
 *
 * Sensor → board orientation:
 *   board_X = -sensor_Z   board_Y = -sensor_X   board_Z = -sensor_Y
 * ======================================================================== */

static void sensors_process_imu(void)
{
  bsp_sensors_axis3i16_t raw_acc, raw_gyro;
  int16_t temp_raw;

  if (!bsp_sensors_read_imu_raw(&raw_acc, &raw_gyro, &temp_raw)) {
    return;
  }

  acc_raw_val.x = -raw_acc.z;
  acc_raw_val.y = -raw_acc.x;
  acc_raw_val.z = -raw_acc.y;

  gyro_raw_val.x = raw_gyro.z;
  gyro_raw_val.y = raw_gyro.x;
  gyro_raw_val.z = raw_gyro.y;

  sensors_calibrate_gyro(gyro_raw_val.x, gyro_raw_val.y, gyro_raw_val.z);

  sensors_data.gyro.x =
    (float)(gyro_raw_val.x - (int16_t)gyro_bias_val.x) * SENSORS_DEG_PER_LSB;
  sensors_data.gyro.y =
    (float)(gyro_raw_val.y - (int16_t)gyro_bias_val.y) * SENSORS_DEG_PER_LSB;
  sensors_data.gyro.z =
    (float)(gyro_raw_val.z - (int16_t)gyro_bias_val.z) * SENSORS_DEG_PER_LSB;

  sensors_apply_lpf_3axis(gyro_lpf, &sensors_data.gyro);

  sensors_data.acc.x = (float)acc_raw_val.x * SENSORS_G_PER_LSB / acc_scale_val;
  sensors_data.acc.y = (float)acc_raw_val.y * SENSORS_G_PER_LSB / acc_scale_val;
  sensors_data.acc.z = (float)acc_raw_val.z * SENSORS_G_PER_LSB / acc_scale_val;

  sensors_apply_lpf_3axis(acc_lpf, &sensors_data.acc);

  if (gyro_bias_found_val) {
    sensors_calibrate_accel(acc_raw_val.x, acc_raw_val.y, acc_raw_val.z);
  }
}

/* ========================================================================
 * Barometer data processing
 *
 * Reads raw pressure/temperature via BSP, runs device-specific
 * compensation math, converts pressure to altitude in cm.
 * ======================================================================== */

static void sensors_process_baro(void)
{
  bsp_sensors_baro_raw_t raw;

  if (!bsp_sensors_read_barometer_raw(&raw)) {
    return;
  }

  if (baro_type == BSP_SENSORS_BAROMETER_BMP280) {
    int32_t temp_x100 = bmp280_compensate_t(raw.temperature);
    uint32_t press_q24_8 = bmp280_compensate_p(raw.pressure);
    sensors_data.baro.temperature = (float)temp_x100 / 100.0f;
    sensors_data.baro.pressure = (float)press_q24_8 / 25600.0f;
  } else if (baro_type == BSP_SENSORS_BAROMETER_SPL06) {
    sensors_data.baro.temperature =
      spl06_compensate_temperature(raw.temperature);
    sensors_data.baro.pressure =
      spl06_compensate_pressure(raw.pressure, raw.temperature) / 100.0f;
  } else {
    return;
  }

  sensors_data.baro.asl =
    pressure_to_altitude(sensors_data.baro.pressure) * 100.0f;
}

/* ========================================================================
 * 3-axis low-pass filter helper
 * ======================================================================== */

static void sensors_apply_lpf_3axis(lpf2p_data_t *lpf, axis3f_t *in)
{
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2p_apply(&lpf[i], in->axis[i]);
  }
}

/* ========================================================================
 * Public API — initialization
 * ======================================================================== */

/** @brief  See sensors.h */
void sensors_init(void)
{
  if (sensors_is_init) {
    return;
  }

  accel_queue = xQueueCreate(1, sizeof(axis3f_t));
  gyro_queue = xQueueCreate(1, sizeof(axis3f_t));
  baro_queue = xQueueCreate(1, sizeof(baro_t));
  data_ready_sem = xSemaphoreCreateBinary();

  bsp_sensors_get_status(&dev_status);
  baro_type = dev_status.barometer_type;

  sensors_bias_obj_init(&gyro_bias_running);

  for (uint8_t i = 0; i < 3; i++) {
    lpf2p_init(&gyro_lpf[i], IMU_SAMPLE_RATE, GYRO_LPF_CUTOFF_FREQ);
    lpf2p_init(&acc_lpf[i], IMU_SAMPLE_RATE, ACCEL_LPF_CUTOFF_FREQ);
  }

  if (baro_type == BSP_SENSORS_BAROMETER_BMP280) {
    sensors_load_bmp280_calib();
  } else if (baro_type == BSP_SENSORS_BAROMETER_SPL06) {
    sensors_load_spl06_calib();
  }

  sensors_is_init = true;
}

/* ========================================================================
 * Public API — tests and queries
 * ======================================================================== */

/** @brief  See sensors.h */
bool sensors_test(void)
{
  if (!sensors_is_init) {
    return false;
  }
  return dev_status.imu_present;
}

/** @brief  See sensors.h */
bool sensors_are_calibrated(void)
{
  return gyro_bias_found_val;
}

/** @brief  See sensors.h */
bool sensors_is_mpu_present(void)
{
  return dev_status.imu_present;
}

/** @brief  See sensors.h */
bool sensors_is_baro_present(void)
{
  return dev_status.barometer_present;
}

/** @brief  See sensors.h */
void sensors_get_raw_data(axis3i16_t *acc, axis3i16_t *gyro, axis3i16_t *mag)
{
  *acc = acc_raw_val;
  *gyro = gyro_raw_val;
  memset(mag, 0, sizeof(*mag));
}

/* ========================================================================
 * Public API — sensor data access
 * ======================================================================== */

/** @brief  See sensors.h */
bool sensors_read_gyro(axis3f_t *gyro)
{
  return xQueueReceive(gyro_queue, gyro, 0) == pdTRUE;
}

/** @brief  See sensors.h */
bool sensors_read_acc(axis3f_t *acc)
{
  return xQueueReceive(accel_queue, acc, 0) == pdTRUE;
}

/** @brief  See sensors.h */
bool sensors_read_baro(baro_t *baro)
{
  return xQueueReceive(baro_queue, baro, 0) == pdTRUE;
}

/** @brief  See sensors.h */
void sensors_acquire(sensor_data_t *out, uint32_t tick)
{
  (void)tick;
  sensors_read_gyro(&out->gyro);
  sensors_read_acc(&out->acc);
  sensors_read_baro(&out->baro);
}

/* ========================================================================
 * Public API — FreeRTOS task entry
 *
 * Wakes on DRDY interrupt, processes raw IMU + baro, publishes filtered
 * and calibrated data to queues for downstream consumers.
 * ======================================================================== */

/** @brief  See sensors.h */
void sensors_task(void *arg)
{
  (void)arg;

  sensors_init();
  vTaskDelay(pdMS_TO_TICKS(150));

  uint32_t tick = 0;

  for (;;) {
    if (xSemaphoreTake(data_ready_sem, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    sensors_process_imu();

    if (dev_status.barometer_present &&
        RATE_DO_EXECUTE(BARO_UPDATE_RATE, tick)) {
      sensors_process_baro();
    }

    xQueueOverwrite(gyro_queue, &sensors_data.gyro);
    xQueueOverwrite(accel_queue, &sensors_data.acc);
    if (dev_status.barometer_present) {
      xQueueOverwrite(baro_queue, &sensors_data.baro);
    }

    tick++;
  }
}
