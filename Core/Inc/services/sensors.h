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

#include "services/sensors_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize sensor subsystem: queues, LPFs, barometer calibration.
 *
 * Safe to call multiple times (idempotent).
 */
void sensors_init(void);

/**
 * @brief  Self-test: returns true if the IMU is present and init completed.
 */
bool sensors_test(void);

/**
 * @brief  Check if gyro bias calibration has completed.
 *
 * @return true if the quad was stationary long enough for bias to converge.
 */
bool sensors_are_calibrated(void);

/**
 * @brief  FreeRTOS task entry -- sensor processing loop.
 *
 * Waits on data_ready_sem (posted by EXTI4 DRDY ISR), processes IMU
 * at 1 kHz and barometer at BARO_UPDATE_RATE.  Publishes filtered data
 * to gyro/accel/baro queues via xQueueOverwrite.
 *
 * @param[in] arg  Unused.
 */
void sensors_task(void *arg);

/**
 * @brief  Read latest gyro, accel, and baro from their queues.
 *
 * @param[out] out   Destination for latest sensor data.
 * @param[in]  tick  Current main-loop tick (unused, reserved for rate filtering).
 */
void sensors_acquire(sensor_data_t *out, uint32_t tick);

/**
 * @brief  Non-blocking read of latest filtered gyro from queue.
 *
 * @param[out] gyro  Destination for gyro data (deg/s).
 * @return true if new data was available.
 */
bool sensors_read_gyro(Axis3f *gyro);

/**
 * @brief  Non-blocking read of latest filtered accelerometer from queue.
 *
 * @param[out] acc  Destination for accel data (g).
 * @return true if new data was available.
 */
bool sensors_read_acc(Axis3f *acc);

/**
 * @brief  Non-blocking read of latest barometer from queue.
 *
 * @param[out] baro  Destination for baro data.
 * @return true if new data was available.
 */
bool sensors_read_baro(baro_t *baro);

/**
 * @brief  Copy current raw sensor values (for diagnostics/debug).
 *
 * @param[out] acc   Raw accelerometer (board frame, LSB).
 * @param[out] gyro  Raw gyroscope (board frame, LSB).
 * @param[out] mag   Zeroed (magnetometer not populated).
 */
void sensors_get_raw_data(Axis3i16 *acc, Axis3i16 *gyro, Axis3i16 *mag);

/**
 * @brief  Query whether the MPU6500 IMU was detected at init.
 */
bool sensors_is_mpu_present(void);

/**
 * @brief  Query whether a barometer (BMP280 or SPL06) was detected at init.
 */
bool sensors_is_baro_present(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_SENSORS_H */
