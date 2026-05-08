// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Sensor data acquisition and processing pipeline.
 *
 * @details
 * Reads raw IMU and barometer data via BSP drivers, applies gyro bias
 * calibration, accelerometer scale correction, low-pass filtering, and
 * barometer compensation.  Filtered data is published to FreeRTOS queues
 * for downstream consumers (stabilizer, estimator).
 *
 * The sensors_task wakes on the MPU6500 DRDY EXTI interrupt and runs at
 * 1 kHz (IMU) with barometer decimated to 50 Hz.
 */

#ifndef SERVICES_SENSORS_H
#define SERVICES_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

#include "services/sensors_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize sensor subsystem: queues, LPFs, barometer calibration.
 *
 * Detects barometer type, loads per-chip calibration coefficients, and
 * initializes the gyro bias ring buffer.  Safe to call multiple times
 * (idempotent after first call).
 *
 * @pre    FreeRTOS scheduler must be running; hi2c1 must be initialized.
 * @warning Must be called before sensors_task is started.
 */
void sensors_init(void);

/**
 * @brief  Self-test: returns true if the IMU is present and init completed.
 *
 * @return true  IMU detected and subsystem initialized.
 * @return false sensors_init has not been called or IMU is absent.
 */
bool sensors_test(void);

/**
 * @brief  Check if gyro bias calibration has completed.
 *
 * Calibration is performed inline during sensors_task execution.  The quad
 * must remain stationary for approximately 1 second (1024 samples) until
 * per-axis variance drops below GYRO_VARIANCE_BASE.
 *
 * @return true  Bias converged; gyro output is bias-corrected.
 * @return false Still collecting samples or motion was detected.
 */
bool sensors_are_calibrated(void);

/**
 * @brief  FreeRTOS task entry -- sensor processing loop.
 *
 * Waits on data_ready_sem (posted by EXTI4 DRDY ISR), processes IMU
 * at 1 kHz and barometer at BARO_UPDATE_RATE.  Publishes filtered data
 * to gyro/accel/baro queues via xQueueOverwrite.
 *
 * @pre    sensors_init() must have been called or will be called internally
 *         at task startup.
 * @param[in] arg  Unused; pass NULL.
 */
void sensors_task(void *arg);

/**
 * @brief  Read latest gyro, accel, and baro from their queues.
 *
 * Convenience wrapper around sensors_read_gyro/acc/baro.  Fields that have
 * no new data since the last call are left unchanged in @p out.
 *
 * @param[out] out   Destination for latest sensor data.
 * @param[in]  tick  Reserved for future rate-gate use; currently unused.
 */
void sensors_acquire(sensor_data_t *out, uint32_t tick);

/**
 * @brief  Non-blocking read of latest filtered gyro from queue.
 *
 * @param[out] gyro  Destination for gyro data (deg/s, board frame).
 * @return true  New data was available and copied to @p gyro.
 * @return false Queue was empty; @p gyro is unchanged.
 */
bool sensors_read_gyro(axis3f_t *gyro);

/**
 * @brief  Non-blocking read of latest filtered accelerometer from queue.
 *
 * @param[out] acc  Destination for accel data (g, board frame).
 * @return true  New data was available and copied to @p acc.
 * @return false Queue was empty; @p acc is unchanged.
 */
bool sensors_read_acc(axis3f_t *acc);

/**
 * @brief  Non-blocking read of latest barometer data from queue.
 *
 * @param[out] baro  Destination for baro data (temperature, pressure, ASL).
 * @return true  New data was available and copied to @p baro.
 * @return false Queue was empty or barometer absent; @p baro is unchanged.
 */
bool sensors_read_baro(baro_t *baro);

/**
 * @brief  Copy current raw sensor values (for diagnostics/debug only).
 *
 * Values are in board frame after axis remapping, before any filtering
 * or calibration.  Units are raw LSB.
 *
 * @param[out] acc   Raw accelerometer (board frame, LSB).
 * @param[out] gyro  Raw gyroscope (board frame, LSB).
 * @param[out] mag   Always zeroed -- magnetometer not populated.
 */
void sensors_get_raw_data(axis3i16_t *acc, axis3i16_t *gyro, axis3i16_t *mag);

/**
 * @brief  Query whether the MPU6500 IMU was detected at init.
 *
 * @return true  IMU responded on SPI during bsp_sensors_get_status.
 * @return false IMU absent or not yet initialized.
 */
bool sensors_is_mpu_present(void);

/**
 * @brief  Query whether a barometer (BMP280 or SPL06) was detected at init.
 *
 * @return true  Barometer responded on I2C1 during bsp_sensors_get_status.
 * @return false Barometer absent or not yet initialized.
 */
bool sensors_is_baro_present(void);

/**
 * @brief  EXTI callback for MPU6500 DRDY interrupt on PA4.
 *
 * Called from the platform IRQ dispatcher inside HAL_GPIO_EXTI_Callback.
 * Posts data_ready_sem to wake sensors_task.
 *
 * @param[in] pin  GPIO pin mask from the EXTI handler; only GPIO_PIN_4 is acted on.
 * @warning Must be called from ISR context only.
 */
void sensors_exti_callback(uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_SENSORS_H */
