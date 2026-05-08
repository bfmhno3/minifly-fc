// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Automated 360-degree flip manoeuvre
 *
 * @details
 * Implements a trapezoidal angular-rate flip profile: accelerate rotation,
 * hold at peak rate, decelerate back to level.  Thrust is adjusted
 * throughout to maintain altitude.  The flip runs as a state machine
 * checked at 500 Hz from the stabilizer loop.
 */

#ifndef CONTROL_FLIP_H
#define CONTROL_FLIP_H

#include <stdbool.h>

#include "control/flight_types.h"

void flip_init(void);

/** @brief  Set the desired flip direction (FLIP_DIR_CENTER to cancel) */
void flip_set_dir(flip_dir_e dir);

/**
 * @brief  Run one tick of the flip state machine
 *
 * Called at 500 Hz from the stabilizer.  Modifies setpoint and control
 * outputs when a flip is in progress.  No-op when idle.
 *
 * @param[in,out] sp    Setpoint (thrust and attitude are overridden during flip)
 * @param[in,out] ctl   Control output (flip_dir is cleared on completion)
 * @param[in]     state Current fused state (used for velocity and attitude checks)
 */
void flip_check(setpoint_t *sp, control_t *ctl, const state_t *state);

#endif /* CONTROL_FLIP_H */
