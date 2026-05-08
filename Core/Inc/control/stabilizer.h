// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Top-level stabilizer — owns the main flight control loop
 *
 * @details
 * The stabilizer task runs at 500 Hz and orchestrates the full control
 * pipeline: sensor acquisition, attitude/position estimation, setpoint
 * resolution, anomaly detection, PID control, and motor output.
 */

#ifndef CONTROL_STABILIZER_H
#define CONTROL_STABILIZER_H

#include <stdbool.h>
#include <stdint.h>

#include "control/flight_types.h"
#include "platform/axis.h"
#include "services/sensors_types.h"

void stabilizer_init(void);
bool stabilizer_test(void);

/**
 * @brief  Main flight control task (runs in a FreeRTOS task)
 *
 * Waits for sensor calibration, then enters the infinite control loop.
 * Pipeline stages run at different sub-rates driven by a shared tick counter.
 *
 * @param[in] arg  Unused (FreeRTOS task signature)
 */
void stabilizer_task(void *arg);

/** @brief  Get the latest attitude estimate (signs flipped for telemetry) */
void stabilizer_get_attitude(attitude_t *out);

/** @brief  Get the latest barometer altitude (meters ASL) */
float stabilizer_get_baro(void);

/** @brief  Snapshot of the most recent raw sensor data */
void stabilizer_get_sensor_data(sensor_data_t *out);

/** @brief  Snapshot of acceleration, velocity, and position state */
void stabilizer_get_state(axis3f_t *acc, axis3f_t *vel, axis3f_t *pos);

/**
 * @brief  Start a fast position-adjust manoeuvre (velocity then absolute hold)
 *
 * Used by the commander to quickly snap to a new altitude after takeoff
 * or manual height changes.
 *
 * @param[in] vel_ticks  Number of ticks to run velocity-mode correction
 * @param[in] abs_ticks  Number of ticks to run absolute-position hold after
 * @param[in] height     Target height for the absolute-hold phase (cm)
 */
void stabilizer_set_fast_adjust(uint16_t vel_ticks, uint16_t abs_ticks,
                                float height);

#endif /* CONTROL_STABILIZER_H */
