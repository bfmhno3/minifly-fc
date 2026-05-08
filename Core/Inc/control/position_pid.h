// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Cascaded position PID controller (position outer + velocity inner)
 *
 * @details
 * Outer loop converts position error to velocity setpoint; inner loop
 * converts velocity error to attitude tilt (XY) or thrust (Z).
 * Auto-calibrates the hover thrust base during stable flight.
 */

#ifndef CONTROL_POSITION_PID_H
#define CONTROL_POSITION_PID_H

#include "control/flight_types.h"
#include "services/config_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise position and velocity PID controllers
 * @param[in] vel_dt  Inner-loop (velocity) time step (seconds)
 * @param[in] pos_dt  Outer-loop (position) time step (seconds)
 */
void position_pid_init(float vel_dt, float pos_dt);

/** @brief  Reset all integral and previous-error state */
void position_pid_reset(void);

/** @brief  Load PID gains from configuration */
void position_pid_set_gains(const pid_group_pos_t *gains);

/**
 * @brief  Run one position-control cycle
 *
 * Outer loop: position error -> velocity setpoint.
 * Inner loop: velocity error -> attitude tilt (XY) and thrust (Z).
 *
 * @param[in,out] sp         Setpoint (velocity fields are written by outer loop)
 * @param[in]     state      Current fused state estimate
 * @param[out]    att_out    Desired pitch/roll from velocity controller
 * @param[out]    thrust_out Thrust command including base hover thrust
 */
void position_pid_run(setpoint_t *sp, const state_t *state, attitude_t *att_out,
                      float *thrust_out);

/** @brief  Get the current smoothed hover thrust estimate (LPF output) */
float position_pid_get_althold_thrust(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_POSITION_PID_H */
