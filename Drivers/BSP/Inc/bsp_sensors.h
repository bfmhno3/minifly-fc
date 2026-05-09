// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Sensor bus abstraction for IMU, magnetometer, and barometer.
 *
 * @details
 * Single-threaded sensor probe and data-read interface for I2C bus 1.
 * Detects MPU6500 (IMU), AK8963 (magnetometer via MPU I2C bypass),
 * and one of BMP280/SPL06 (barometer).
 *
 * Device discovery results are cached in module-private state; callers
 * query detected devices via bsp_sensors_get_status().
 *
 * Non-thread-safe: if using in RTOS context, external synchronization required.
 */

#ifndef BSP_SENSORS_H
#define BSP_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Signed 3-axis raw reading (accelerometer, gyroscope, or magnetometer).
 */
typedef struct bsp_sensors_axis3i16 {
  int16_t x;
  int16_t y;
  int16_t z;
} bsp_sensors_axis3i16_t;

/**
 * @brief Raw barometer reading before compensation (ADC counts, device-specific encoding).
 */
typedef struct bsp_sensors_baro_raw {
  int32_t pressure;    /**< Raw pressure (device-specific encoding). */
  int32_t temperature; /**< Raw temperature (device-specific encoding). */
} bsp_sensors_baro_raw_t;

/**
 * @brief Detected barometer IC type.
 */
typedef enum bsp_sensors_barometer_type {
  BSP_SENSORS_BAROMETER_NONE = 0,
  BSP_SENSORS_BAROMETER_BMP280 = 1,
  BSP_SENSORS_BAROMETER_SPL06 = 2,
} bsp_sensors_barometer_type_t;

/**
 * @brief Probe result — which sensors are present on the I2C bus.
 */
typedef struct {
  bool imu_present;
  bool magnetometer_present;
  bool barometer_present;
  bsp_sensors_barometer_type_t barometer_type;
  uint8_t
    imu_address; /**< 7-bit I2C address of the detected IMU (0x68 or 0x69). */
} bsp_sensors_status_t;

/**
 * @brief Probe I2C bus 1 for all sensors and initialize detected devices.
 *
 * Attempts IMU at 0x69 then 0x68, then AK8963 magnetometer (behind MPU bypass),
 * then BMP280 or SPL06 barometer. Stores probe results in module context.
 *
 * Safe to call multiple times (re-probes each time).
 */
void bsp_sensors_init(void);

/**
 * @brief Query whether bsp_sensors_init() has been invoked at least once.
 *
 * @retval true if init has been called (probe may have failed).
 * @retval false if init has never been called.
 */
bool bsp_sensors_is_initialized(void);

/**
 * @brief Copy the current probe result (device presence and addresses).
 *
 * @param[out] status Probe snapshot; unchanged if status == NULL.
 */
void bsp_sensors_get_status(bsp_sensors_status_t *status);

/**
 * @brief Check if fresh IMU data is ready for pickup.
 *
 * Reads both PA4 DRDY GPIO (MPU INT output) and MPU interrupt status register.
 * Both must assert data-ready for this function to return true.
 *
 * @retval true  Data ready (GPIO high AND interrupt status bit set).
 * @retval false Not ready, not initialized, or I2C read failed.
 */
bool bsp_sensors_is_data_ready(void);

/**
 * @brief Burst-read accelerometer, gyroscope, and temperature from MPU6500.
 *
 * Single 14-byte I2C read from ACCEL_XOUT_H (big-endian, MPU format).
 *
 * @param[out] accelerometer  Raw counts, +/-16g full-scale mapped to int16_t.
 * @param[out] gyroscope      Raw counts, +/-2000 dps full-scale mapped to int16_t.
 * @param[out] temperature_raw  Raw ADC value; use datasheet formula T_celsius = (raw / 333.87) + 21.
 *
 * @retval true  I2C transfer succeeded.
 * @retval false Not initialized, IMU absent, or I2C error.
 */
bool bsp_sensors_read_imu_raw(bsp_sensors_axis3i16_t *accelerometer,
                              bsp_sensors_axis3i16_t *gyroscope,
                              int16_t *temperature_raw);

/**
 * @brief Read magnetometer data from AK8963 via MPU I2C bypass.
 *
 * Single 7-byte I2C read: ST1 status + 6 data bytes + ST2 status (little-endian data).
 * Only succeeds if DRDY bit in ST1 is set and no overflow/error in ST2.
 *
 * @param[out] magnetometer  Raw counts, +/-4912 uT (typical) mapped to int16_t.
 *
 * @retval true  Data valid and no overflow/error flagged.
 * @retval false DRDY not asserted, overflow/error flag set, or I2C error.
 */
bool bsp_sensors_read_magnetometer_raw(bsp_sensors_axis3i16_t *magnetometer);

/**
 * @brief Read raw pressure and temperature from the detected barometer (BMP280 or SPL06).
 *
 * Dispatches to device-specific decoder based on barometer_type from init.
 * Handles different byte layouts: BMP280 is 20-bit MSB-aligned,
 * SPL06 is 24-bit two's complement big-endian.
 *
 * @param[out] barometer  Raw ADC values; encoding is device-specific (use calibration coefficients).
 *
 * @retval true  I2C transfer succeeded.
 * @retval false No barometer detected, not initialized, or I2C error.
 */
bool bsp_sensors_read_barometer_raw(bsp_sensors_baro_raw_t *barometer);

#ifdef __cplusplus
}
#endif

#endif
