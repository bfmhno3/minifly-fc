// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Sensor data types and scheduling rate definitions.
 *
 * @details
 * Defines the barometer and composite sensor data structures used throughout
 * the sensor pipeline, along with rate constants and a tick-based scheduling
 * macro (RATE_DO_EXECUTE) for decimating sub-loop tasks from the main loop.
 */

#ifndef SERVICES_SENSORS_TYPES_H
#define SERVICES_SENSORS_TYPES_H

#include "platform/axis.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Compensated barometer readings.
 */
typedef struct {
  float pressure;    /**< Atmospheric pressure (hPa) */
  float temperature; /**< Ambient temperature (deg C) */
  float asl;         /**< Altitude above sea level (cm) */
} baro_t;

/**
 * @brief  Composite sensor sample: IMU axes + barometer.
 *
 * Populated by sensors_task and published via queues.
 */
typedef struct {
  Axis3f acc;  /**< Accelerometer (g) */
  Axis3f gyro; /**< Gyroscope (deg/s) */
  Axis3f mag;  /**< Magnetometer (unused placeholder) */
  baro_t baro; /**< Barometer */
} sensor_data_t;

/** @name Task scheduling rates (Hz)
 * @{ */
#define RATE_5_HZ 5
#define RATE_10_HZ 10
#define RATE_25_HZ 25
#define RATE_50_HZ 50
#define RATE_100_HZ 100
#define RATE_200_HZ 200
#define RATE_250_HZ 250
#define RATE_500_HZ 500
#define RATE_1000_HZ 1000
/** @} */

#define MAIN_LOOP_RATE RATE_1000_HZ           /**< Main loop tick rate (Hz) */
#define MAIN_LOOP_DT (1000U / MAIN_LOOP_RATE) /**< Main loop period (ms) */

/**
 * @brief  Returns true every (MAIN_LOOP_RATE / RATE_HZ) ticks.
 *
 * Used to decimate slower sub-tasks (e.g., barometer at 50 Hz)
 * from the 1 kHz main loop.
 */
#define RATE_DO_EXECUTE(RATE_HZ, TICK) \
  ((TICK) % (MAIN_LOOP_RATE / (RATE_HZ)) == 0U)

#define BARO_UPDATE_RATE RATE_50_HZ     /**< Barometer polling rate (Hz) */
#define SENSOR9_UPDATE_RATE RATE_500_HZ /**< 9-axis fusion update rate (Hz) */
#define SENSOR9_UPDATE_DT \
  (1.0f / SENSOR9_UPDATE_RATE) /**< 9-axis update period (s) */

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_SENSORS_TYPES_H */
