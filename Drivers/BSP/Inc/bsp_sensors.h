// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Sensor bus abstraction for IMU, magnetometer, and barometer.
 *
 * @details
 * Probes I2C bus 1 for MPU6500 (or compatible), AK8963 magnetometer, and
 * one of BMP280 / SPL06 barometers.  The detected device set is stored in
 * a module-private context; callers query it via bsp_sensors_get_status().
 */

#ifndef BSP_SENSORS_H
#define BSP_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Signed 3-axis raw reading (accelerometer, gyroscope, or magnetometer). */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} bsp_sensors_axis3i16_t;

/** @brief Raw barometer reading before compensation. */
typedef struct
{
    int32_t pressure;    /**< Raw pressure (device-specific encoding). */
    int32_t temperature; /**< Raw temperature (device-specific encoding). */
} bsp_sensors_baro_raw_t;

/** @brief Detected barometer IC type. */
typedef enum
{
    BSP_SENSORS_BAROMETER_NONE = 0,
    BSP_SENSORS_BAROMETER_BMP280 = 1,
    BSP_SENSORS_BAROMETER_SPL06 = 2,
} bsp_sensors_barometer_type_t;

/** @brief Probe result — which sensors are present on the I2C bus. */
typedef struct
{
    bool imu_present;
    bool magnetometer_present;
    bool barometer_present;
    bsp_sensors_barometer_type_t barometer_type;
    uint8_t imu_address; /**< 7-bit I2C address of the detected IMU (0x68 or 0x69). */
} bsp_sensors_status_t;

/**
 * @brief Probe I2C bus and initialize all detected sensors.
 *
 * Attempts MPU at 0x69 then 0x68, then AK8963 (behind MPU bypass),
 * then BMP280 or SPL06 barometer.  Stores results in module context.
 */
void bsp_sensors_init(void);

/**
 * @brief Check whether bsp_sensors_init() has been called.
 *
 * @return true if the sensor subsystem has been probed at least once.
 */
bool bsp_sensors_is_initialized(void);

/**
 * @brief Copy the current sensor probe status into @p status.
 *
 * @param[out] status  Destination for the probe result.
 */
void bsp_sensors_get_status(bsp_sensors_status_t *status);

/**
 * @brief Check if new IMU data is available.
 *
 * Reads the DRDY GPIO (PA4) and the MPU interrupt status register.
 *
 * @retval true  Both GPIO and register indicate data ready.
 * @retval false Not ready, not initialized, or read failure.
 */
bool bsp_sensors_is_data_ready(void);

/**
 * @brief Burst-read accelerometer, gyroscope, and temperature from the IMU.
 *
 * Reads 14 bytes starting at ACCEL_XOUT_H (big-endian, MPU register map).
 *
 * @param[out] accelerometer  Raw accel counts (X, Y, Z).
 * @param[out] gyroscope      Raw gyro counts (X, Y, Z).
 * @param[out] temperature_raw  Raw temperature (convert with datasheet formula).
 *
 * @retval true  Read succeeded.
 * @retval false Not initialized, IMU absent, or I2C failure.
 */
bool bsp_sensors_read_imu_raw(
    bsp_sensors_axis3i16_t *accelerometer,
    bsp_sensors_axis3i16_t *gyroscope,
    int16_t *temperature_raw);

/**
 * @brief Read magnetometer data from AK8963 via MPU bypass.
 *
 * Reads ST1 + 6 data bytes + ST2 in a single I2C transaction (little-endian).
 *
 * @param[out] magnetometer  Raw mag counts (X, Y, Z).
 *
 * @retval true  Read succeeded and data is valid.
 * @retval false Not ready, overflow/error flag set, or I2C failure.
 */
bool bsp_sensors_read_magnetometer_raw(bsp_sensors_axis3i16_t *magnetometer);

/**
 * @brief Read raw pressure and temperature from the detected barometer.
 *
 * Dispatches to BMP280 or SPL06 depending on which was detected at init.
 *
 * @param[out] barometer  Raw pressure and temperature (device-specific encoding).
 *
 * @retval true  Read succeeded.
 * @retval false No barometer detected, not initialized, or I2C failure.
 */
bool bsp_sensors_read_barometer_raw(bsp_sensors_baro_raw_t *barometer);

#ifdef __cplusplus
}
#endif

#endif
