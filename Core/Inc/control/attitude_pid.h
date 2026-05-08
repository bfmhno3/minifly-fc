// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Cascaded attitude PID controller (angle outer loop + rate inner loop)
 *
 * @details
 * Provides a two-stage PID cascade: the outer loop converts angle error to
 * a desired angular rate, and the inner loop converts rate error to motor
 * effort.  Roll and pitch share symmetric limits; yaw uses wider integral
 * bounds to handle continuous-heading tracking.
 */

#ifndef CONTROL_ATTITUDE_PID_H
#define CONTROL_ATTITUDE_PID_H

#include "platform/axis.h"
#include "control/flight_types.h"
#include "services/config_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise all six PID controllers (3 angle + 3 rate).
 * @param rate_dt  Inner-loop time step (seconds)
 * @param angle_dt Outer-loop time step (seconds)
 */
void attitude_pid_init(float rate_dt, float angle_dt);

/**
 * @brief Reset integral and previous-error state for all six PIDs.
 */
void attitude_pid_reset(void);

/**
 * @brief Set outer-loop (angle) PID gains for all three axes.
 */
void attitude_pid_set_angle_gains(const pid_group_t *gains);

/**
 * @brief Set inner-loop (rate) PID gains for all three axes.
 */
void attitude_pid_set_rate_gains(const pid_group_t *gains);

/**
 * @brief Outer loop: angle error -> desired angular rate.
 * @param actual       Current attitude (roll/pitch/yaw, degrees)
 * @param desired      Target attitude (degrees)
 * @param desired_rate Output: desired body angular velocity (deg/s)
 */
void attitude_pid_run_angle(const attitude_t *actual, const attitude_t *desired,
                            axis3f_t *desired_rate);

/**
 * @brief Inner loop: rate error -> control output.
 * @param actual_rate  Measured body angular velocity (deg/s)
 * @param desired_rate Target angular velocity (deg/s)
 * @param control_out  Output: control effort per axis
 */
void attitude_pid_run_rate(const axis3f_t *actual_rate,
                           const axis3f_t *desired_rate, axis3f_t *control_out);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_ATTITUDE_PID_H */
