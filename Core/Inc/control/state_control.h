// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  State control — coordinates the cascaded PID loops
 *
 * @details
 * Runs the three-tier PID cascade: position (250 Hz) -> angle (250 Hz)
 * -> rate (500 Hz).  Selects between position-controlled and manual
 * thrust modes based on the setpoint mode flags.
 */

#ifndef CONTROL_STATE_CONTROL_H
#define CONTROL_STATE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"
#include "services/sensors_types.h"

void state_control_init(void);
bool state_control_test(void);

/**
 * @brief  Run one control cycle (call every tick at 500 Hz)
 *
 * Internally executes position/angle/rate PID loops at their respective
 * sub-rates.  When thrust is below THRUST_OFF_THRESHOLD, all PIDs are
 * reset and outputs are zeroed.
 *
 * @param[out] out     Control output (roll/pitch/yaw effort + thrust)
 * @param[in]  sensor  Raw sensor data (gyro used for rate loop)
 * @param[in]  state   Fused state estimate
 * @param[in]  sp      Commanded setpoint
 * @param[in]  tick    Monotonic tick counter for sub-rate scheduling
 */
void state_control_run(control_t *out, const sensor_data_t *sensor,
		       const state_t *state, const setpoint_t *sp,
		       uint32_t tick);

#endif /* CONTROL_STATE_CONTROL_H */
