// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  PMW3901 optical flow sensor expansion module implementation.
 *
 * @details
 * Communicates with a PMW3901 optical flow sensor over SPI2.  A dedicated
 * FreeRTOS task reads motion burst data at 100 Hz.  The public update
 * function converts raw pixel deltas to velocity/position estimates using
 * the current height and attitude for compensation.
 *
 * Hardware dependencies:
 * - SPI2 (CubeMX-initialized) for PMW3901 register and burst access.
 * - NCS on PA8 (shared with MODULE_SHARED_SCL, reconfigured as GPIO output).
 * - Power on PB0 (shared module power pin).
 */

#include "modules/optical_flow_module.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control/position_estimator.h"
#include "platform/axis.h"
#include "spi.h"
#include "gpio.h"
#include "main.h"

/* --- PMW3901 optical flow sensor constants --- */

#define OPFLOW_RESOLUTION \
  0.2131946f /**< pixel-to-cm conversion at 1 m height */
#define OPFLOW_OUTLIER_LIMIT \
  100 /**< reject single-frame deltas above this (pixels) */
#define OPFLOW_VEL_LIMIT 150.0f   /**< velocity clamp, cm/s */
#define OPFLOW_LPF_ALPHA 0.15f    /**< velocity low-pass filter coefficient */
#define OPFLOW_MAX_HEIGHT_M 4.0f  /**< sensor operating ceiling, metres */
#define OPFLOW_MIN_HEIGHT_M 0.05f /**< sensor operating floor, metres */
#define OPFLOW_INVALID_TIMEOUT \
  100 /**< consecutive invalid reads before declaring data stale */
#define OPFLOW_TASK_PERIOD_MS 10 /**< sampling task period (100 Hz) */
#define OPFLOW_ATTITUDE_COMP \
  480.0f /**< attitude compensation gain (pixels per tan(radian)) */

#define DEG2RAD 0.01745329251994f /**< degrees to radians */

#define X 0
#define Y 1

/* NCS pin on PA8, active low (shared with MODULE_SHARED_SCL, reconfigured here) */
#define NCS_PORT MODULE_SHARED_SCL_GPIO_Port
#define NCS_PIN MODULE_SHARED_SCL_Pin

/* power enable pin on PB0 */
#define POWER_PORT MODULE_POWER_GPIO_Port
#define POWER_PIN MODULE_POWER_Pin

/* --- motion burst data from PMW3901 (12 bytes on wire) --- */

typedef struct __packed {
  uint8_t motion;
  uint8_t observation;
  int16_t delta_x;
  int16_t delta_y;
  uint8_t squal;
  uint8_t raw_data_sum;
  uint8_t max_raw_data;
  uint8_t min_raw_data;
  uint16_t shutter;
} motion_burst_t;

/* --- module internal state --- */

static bool is_init;
static bool is_sensor_ok;
static bool is_data_valid;
static uint8_t invalid_cnt;

static float pix_sum[2];        /**< accumulated raw pixel motion [x, y] */
static float pix_comp[2];       /**< attitude compensation offset [x, y] */
static float pix_valid[2];      /**< compensated pixel position [x, y] */
static float pix_valid_last[2]; /**< previous compensated position [x, y] */
static float delta_pos[2];      /**< position delta this step, cm [x, y] */
static float delta_vel[2];      /**< instantaneous velocity, cm/s [x, y] */
static float pos_sum[2];        /**< accumulated position, cm [x, y] */
static float vel_lpf[2];        /**< low-pass filtered velocity, cm/s [x, y] */

static TaskHandle_t task_handle;
static motion_burst_t current_motion;

/* --- helper functions --- */

/**
 * @brief  Clamp a float to [min, max].
 */
static float constrainf(float val, float min, float max)
{
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

static void ncs_low(void)
{
  HAL_GPIO_WritePin(NCS_PORT, NCS_PIN, GPIO_PIN_RESET);
}

static void ncs_high(void)
{
  HAL_GPIO_WritePin(NCS_PORT, NCS_PIN, GPIO_PIN_SET);
}

static void power_set(bool on)
{
  HAL_GPIO_WritePin(POWER_PORT, POWER_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* --- PMW3901 SPI register access --- */

/**
 * @brief  Write a single register on the PMW3901.
 *
 * PMW3901 SPI protocol: bit7 of the address byte = 1 for write.
 */
static void reg_write(uint8_t reg, uint8_t val)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80u; /* bit7=1 for write */
  tx[1] = val;

  ncs_low();
  HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);
  ncs_high();
}

/**
 * @brief  Read a single register from the PMW3901.
 *
 * PMW3901 SPI protocol: bit7 of the address byte = 0 for read.
 */
static uint8_t reg_read(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg & ~0x80u; /* bit7=0 for read */
  tx[1] = 0;

  ncs_low();
  HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);
  ncs_high();

  return rx[1];
}

/**
 * @brief  Read a 12-byte motion burst from the PMW3901.
 *
 * The motion burst register (0x16) returns the latest accumulated motion
 * data in a single SPI transaction, avoiding the overhead of per-register
 * reads.  The shutter field arrives big-endian and is swapped to match
 * the little-endian struct layout.
 */
static void motion_burst_read(motion_burst_t *burst)
{
  uint8_t tx[13];
  uint8_t rx[13];

  memset(tx, 0, sizeof(tx));
  tx[0] = 0x16; /* motion burst register */

  ncs_low();
  HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), HAL_MAX_DELAY);
  ncs_high();

  memcpy(burst, &rx[1], sizeof(motion_burst_t));

  /* fix shutter byte order (big-endian in SPI, little-endian in struct) */
  uint16_t s = burst->shutter;
  burst->shutter = (uint16_t)((s >> 8) & 0x00ff) |
                   (uint16_t)((s & 0x00ff) << 8);
}

/* --- PMW3901 register initialization --- */

/**
 * @brief  Write the PMW3901 register initialization sequence.
 *
 * These register values are taken from the PMW3901 datasheet and
 * application note.  They configure the sensor for indoor use on
 * a typical drone surface (non-glossy, textured).
 */
static void pmw3901_init_registers(void)
{
  reg_write(0x7F, 0x00);
  reg_write(0x61, 0xAD);
  reg_write(0x7F, 0x03);
  reg_write(0x40, 0x00);
  reg_write(0x7F, 0x05);
  reg_write(0x41, 0xB3);
  reg_write(0x43, 0xF1);
  reg_write(0x45, 0x14);
  reg_write(0x5B, 0x32);
  reg_write(0x5F, 0x34);
  reg_write(0x7B, 0x08);
  reg_write(0x7F, 0x06);
  reg_write(0x44, 0x1B);
  reg_write(0x40, 0xBF);
  reg_write(0x4E, 0x3F);
  reg_write(0x7F, 0x08);
  reg_write(0x65, 0x20);
  reg_write(0x6A, 0x18);
  reg_write(0x7F, 0x09);
  reg_write(0x4F, 0xAF);
  reg_write(0x5F, 0x40);
  reg_write(0x48, 0x80);
  reg_write(0x49, 0x80);
  reg_write(0x57, 0x77);
  reg_write(0x60, 0x78);
  reg_write(0x61, 0x78);
  reg_write(0x62, 0x08);
  reg_write(0x63, 0x50);
  reg_write(0x7F, 0x0A);
  reg_write(0x45, 0x60);
  reg_write(0x7F, 0x00);
  reg_write(0x4D, 0x11);
  reg_write(0x55, 0x80);
  reg_write(0x74, 0x1F);
  reg_write(0x75, 0x1F);
  reg_write(0x4A, 0x78);
  reg_write(0x4B, 0x78);
  reg_write(0x44, 0x08);
  reg_write(0x45, 0x50);
  reg_write(0x64, 0xFF);
  reg_write(0x65, 0x1F);
  reg_write(0x7F, 0x14);
  reg_write(0x65, 0x67);
  reg_write(0x66, 0x08);
  reg_write(0x63, 0x70);
  reg_write(0x7F, 0x15);
  reg_write(0x48, 0x48);
  reg_write(0x7F, 0x07);
  reg_write(0x41, 0x0D);
  reg_write(0x43, 0x14);
  reg_write(0x4B, 0x0E);
  reg_write(0x45, 0x0F);
  reg_write(0x44, 0x42);
  reg_write(0x4C, 0x80);
  reg_write(0x7F, 0x10);
  reg_write(0x5B, 0x02);
  reg_write(0x7F, 0x07);
  reg_write(0x40, 0x41);
  reg_write(0x70, 0x00);

  vTaskDelay(pdMS_TO_TICKS(10));

  reg_write(0x32, 0x44);
  reg_write(0x7F, 0x07);
  reg_write(0x40, 0x40);
  reg_write(0x7F, 0x06);
  reg_write(0x62, 0xF0);
  reg_write(0x63, 0x00);
  reg_write(0x7F, 0x0D);
  reg_write(0x48, 0xC0);
  reg_write(0x6F, 0xD5);
  reg_write(0x7F, 0x00);
  reg_write(0x5B, 0xA0);
  reg_write(0x4E, 0xA8);
  reg_write(0x5A, 0x50);
  reg_write(0x40, 0x80);
}

/* --- reset pixel accumulators --- */

/**
 * @brief  Reset all pixel accumulators to zero.
 */
static void reset_pixel_data(void)
{
  for (int i = 0; i < 2; i++) {
    pix_sum[i] = 0.0f;
    pix_comp[i] = 0.0f;
    pix_valid[i] = 0.0f;
    pix_valid_last[i] = 0.0f;
  }
}

/* --- optical flow FreeRTOS task (100Hz) --- */

/**
 * @brief  FreeRTOS task: reads PMW3901 motion bursts at 100 Hz.
 *
 * Accumulates pixel deltas (with outlier rejection) into pix_sum[].
 * If the sensor returns all-zero data for 100 consecutive reads, the
 * task suspends itself to avoid wasting CPU on a disconnected sensor.
 */
static void optical_flow_task(void *arg)
{
  (void)arg;

  static uint16_t no_data_cnt = 0;
  uint32_t last_wake = xTaskGetTickCount();

  is_sensor_ok = true;

  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(OPFLOW_TASK_PERIOD_MS));

    motion_burst_read(&current_motion);

    if (current_motion.min_raw_data == 0 && current_motion.max_raw_data == 0) {
      if (++no_data_cnt > 100 && is_sensor_ok) {
        no_data_cnt = 0;
        is_sensor_ok = false;
        vTaskSuspend(task_handle);
      }
    } else {
      no_data_cnt = 0;
    }

    /* swap axes for physical installation orientation */
    int16_t pixel_dx = current_motion.delta_y;
    int16_t pixel_dy = -current_motion.delta_x;

    if (abs(pixel_dx) < OPFLOW_OUTLIER_LIMIT &&
        abs(pixel_dy) < OPFLOW_OUTLIER_LIMIT) {
      pix_sum[X] += (float)pixel_dx;
      pix_sum[Y] += (float)pixel_dy;
    }
  }
}

/* --- public API (Doxygen in .h) --- */

void optical_flow_module_init(void)
{
  /* configure NCS pin (PA8) as GPIO output */
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = NCS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NCS_PORT, &gpio);

  /* power on the optical flow sensor */
  power_set(true);
  vTaskDelay(pdMS_TO_TICKS(50));

  ncs_high();

  /* SPI2 is already initialized by CubeMX (MX_SPI2_Init) */

  /* PMW3901 power-on reset */
  reg_write(0x3A, 0x5A);
  vTaskDelay(pdMS_TO_TICKS(5));

  pmw3901_init_registers();
  vTaskDelay(pdMS_TO_TICKS(5));

  reset_pixel_data();
  invalid_cnt = 0;
  is_data_valid = false;

  memset(pos_sum, 0, sizeof(pos_sum));
  memset(vel_lpf, 0, sizeof(vel_lpf));

  if (task_handle == NULL) {
    xTaskCreate(optical_flow_task, "OPFLOW", 300, NULL, 4, &task_handle);
  } else {
    vTaskResume(task_handle);
  }

  is_init = true;
}

void optical_flow_module_deinit(void)
{
  if (task_handle != NULL) {
    vTaskSuspend(task_handle);
  }

  power_set(false);
  is_data_valid = false;
}

void optical_flow_module_update(state_t *state, float dt)
{
  float height_m = 0.01f * position_estimator_get_fused_height();

  if (is_sensor_ok && height_m < OPFLOW_MAX_HEIGHT_M) {
    invalid_cnt = 0;
    is_data_valid = true;

    float coeff = OPFLOW_RESOLUTION * height_m;

    float tan_roll = tanf(state->attitude.roll * DEG2RAD);
    float tan_pitch = tanf(state->attitude.pitch * DEG2RAD);

    pix_comp[X] = OPFLOW_ATTITUDE_COMP * tan_pitch;
    pix_comp[Y] = OPFLOW_ATTITUDE_COMP * tan_roll;

    pix_valid[X] = pix_sum[X] + pix_comp[X];
    pix_valid[Y] = pix_sum[Y] + pix_comp[Y];

    if (height_m < OPFLOW_MIN_HEIGHT_M) {
      coeff = 0.0f;
    }

    delta_pos[X] = coeff * (pix_valid[X] - pix_valid_last[X]);
    delta_pos[Y] = coeff * (pix_valid[Y] - pix_valid_last[Y]);

    pix_valid_last[X] = pix_valid[X];
    pix_valid_last[Y] = pix_valid[Y];

    delta_vel[X] = delta_pos[X] / dt;
    delta_vel[Y] = delta_pos[Y] / dt;

    vel_lpf[X] += (delta_vel[X] - vel_lpf[X]) * OPFLOW_LPF_ALPHA;
    vel_lpf[Y] += (delta_vel[Y] - vel_lpf[Y]) * OPFLOW_LPF_ALPHA;

    vel_lpf[X] = constrainf(vel_lpf[X], -OPFLOW_VEL_LIMIT, OPFLOW_VEL_LIMIT);
    vel_lpf[Y] = constrainf(vel_lpf[Y], -OPFLOW_VEL_LIMIT, OPFLOW_VEL_LIMIT);

    pos_sum[X] += delta_pos[X];
    pos_sum[Y] += delta_pos[Y];
  } else if (is_data_valid) {
    if (++invalid_cnt > OPFLOW_INVALID_TIMEOUT) {
      invalid_cnt = 0;
      is_data_valid = false;
    }
    reset_pixel_data();
  }
}

bool optical_flow_module_is_valid(void)
{
  return is_data_valid;
}

void optical_flow_module_get_data(optical_flow_data_t *out)
{
  out->valid = is_data_valid;
  out->pos_sum[X] = pos_sum[X];
  out->pos_sum[Y] = pos_sum[Y];
  out->vel_lpf[X] = vel_lpf[X];
  out->vel_lpf[Y] = vel_lpf[Y];
  out->laser_range = 0.0f;
  out->laser_quality = 0.0f;
}
