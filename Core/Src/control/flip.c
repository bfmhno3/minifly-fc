// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Flip manoeuvre state machine implementation
 *
 * @details
 * Eight-state machine: IDLE -> SET -> SPEED_UP -> SLOW_DOWN -> ROTATE ->
 * FINISHED -> RECOVERY -> IDLE (or ERROR -> IDLE on timeout).
 * Uses a trapezoidal angular-rate profile for the 360-degree rotation.
 */

#include "control/flip.h"
#include "comm/commander.h"
#include "services/config_service.h"
#include <math.h>

/*
 * Flip state machine constants.
 *
 * The flip maneuver runs at FLIP_RATE Hz and uses a trapezoidal
 * angular-rate profile: accelerate to FLIP_MAX_RATE, hold for
 * max_rate_cnt ticks, then decelerate back to zero.
 */
#define FLIP_RATE              500    /* must match stabilizer tick rate */
#define FLIP_MID_ANGLE         (180.0f * FLIP_RATE) /* halfway angle in rate-ticks */
#define FLIP_MAX_RATE          1380   /* peak angular rate (rate-ticks per tick) */
#define FLIP_DELTA_RATE        (30000.0f / FLIP_MAX_RATE) /* accel/decel step per tick */

#define FLIP_TIMEOUT           800    /* max ticks for the entire flip */
#define FLIP_SPEED_UP_TIMEOUT  400    /* max ticks to reach desired climb speed */
#define FLIP_RECOVERY_TIME     160    /* ticks of thrust recovery after flip */

#define FLIP_MIN_THRUST        28000  /* minimum thrust to allow flip */
#define FLIP_MIN_VEL_Z         (-20.0f)  /* max downward velocity to allow flip (cm/s) */
#define FLIP_DESIRED_VEL_Z     105.0f /* target climb velocity before rotation (cm/s) */

enum flip_state_e {
	FLIP_STATE_IDLE = 0,
	FLIP_STATE_SET,
	FLIP_STATE_SPEED_UP,
	FLIP_STATE_SLOW_DOWN,
	FLIP_STATE_ROTATE,
	FLIP_STATE_FINISHED,
	FLIP_STATE_RECOVERY,
	FLIP_STATE_ERROR,
};

static enum flip_state_e flip_state;
static flip_dir_e        flip_dir;

static float    current_rate;
static float    current_angle;
static uint16_t max_rate_cnt;
static uint16_t timeout_cnt;
static uint16_t recovery_cnt;
static uint16_t exit_cnt;
static bool     exit_flip;

/* Saved attitude at flip start */
static float saved_roll;
static float saved_pitch;
static float saved_yaw;

/* Thrust working values */
static uint16_t flip_thrust_base;
static uint16_t flip_thrust_max;
static uint16_t temp_thrust;
static float    delta_thrust;

void flip_init(void)
{
	flip_state     = FLIP_STATE_IDLE;
	flip_dir       = FLIP_DIR_CENTER;
	current_rate   = 0.0f;
	current_angle  = 0.0f;
	max_rate_cnt   = 0;
	timeout_cnt    = 0;
	recovery_cnt   = 0;
	exit_cnt       = 0;
	exit_flip      = true;
}

void flip_set_dir(flip_dir_e dir)
{
	flip_dir = dir;
}

/*
 * State: IDLE
 *
 * Wait for a flip direction to be set. Only allow flip when
 * thrust is above minimum and vertical velocity is acceptable.
 * After a completed flip, hold a cooldown before re-arming.
 */
static void state_idle(const control_t *ctl, const state_t *state)
{
	if (flip_dir != FLIP_DIR_CENTER) {
		if (ctl->thrust > FLIP_MIN_THRUST &&
		    state->velocity.z > FLIP_MIN_VEL_Z) {
			flip_state = FLIP_STATE_SET;
			exit_cnt   = 500;
			exit_flip  = false;
		} else {
			flip_dir = FLIP_DIR_CENTER;
		}
	} else if (!exit_flip) {
		if (exit_cnt > 0)
			exit_cnt--;
		else
			exit_flip = true;
	}
}

/*
 * State: SET
 *
 * Capture current attitude and compute thrust parameters
 * based on the configured thrustBase.
 */
static void state_set(const state_t *state)
{
	const config_param_t *cfg = config_service_get();

	current_rate  = 0.0f;
	current_angle = 0.0f;
	max_rate_cnt  = 0;
	timeout_cnt   = 0;

	flip_thrust_base = (uint16_t)(-9000.0f + 1.2f * cfg->thrustBase);
	delta_thrust     = cfg->thrustBase / 90.0f;

	flip_thrust_max = cfg->thrustBase + 20000;
	if (flip_thrust_max > 62000)
		flip_thrust_max = 62000;

	temp_thrust = flip_thrust_base;

	saved_roll  = state->attitude.roll;
	saved_pitch = state->attitude.pitch;
	saved_yaw   = state->attitude.yaw;

	flip_state = FLIP_STATE_SPEED_UP;
}

/*
 * State: SPEED_UP
 *
 * Increase thrust to gain altitude before the rotation.
 * Transition when vertical velocity reaches target or timeout.
 */
static void state_speed_up(setpoint_t *sp, const state_t *state)
{
	if (state->velocity.z < FLIP_DESIRED_VEL_Z) {
		sp->mode.z = MODE_DISABLE;
		if (temp_thrust < flip_thrust_max)
			temp_thrust += (uint16_t)delta_thrust;
		sp->thrust = temp_thrust;

		if (++timeout_cnt > FLIP_SPEED_UP_TIMEOUT) {
			timeout_cnt = 0;
			flip_state  = FLIP_STATE_SLOW_DOWN;
		}
	} else {
		timeout_cnt = 0;
		flip_state  = FLIP_STATE_SLOW_DOWN;
	}
}

/*
 * State: SLOW_DOWN
 *
 * Reduce thrust to the flip base level before entering rotation.
 */
static void state_slow_down(setpoint_t *sp)
{
	if (temp_thrust > flip_thrust_base) {
		temp_thrust -= (uint16_t)(6500.0f - flip_thrust_base / 10.0f);
		sp->mode.z  = MODE_DISABLE;
		sp->thrust  = temp_thrust;
	} else {
		flip_state = FLIP_STATE_ROTATE;
	}
}

/*
 * State: ROTATE
 *
 * Execute the actual 360-degree rotation using a trapezoidal
 * angular-rate profile. The drone accelerates rotation to
 * FLIP_MAX_RATE, holds, then decelerates. Thrust is adjusted
 * to compensate for the rotation.
 */
static void state_rotate(setpoint_t *sp, control_t *ctl)
{
	if (++timeout_cnt > FLIP_TIMEOUT) {
		timeout_cnt = 0;
		flip_state  = FLIP_STATE_ERROR;
		return;
	}

	sp->mode.z = MODE_DISABLE;
	sp->thrust = flip_thrust_base - 3 * current_rate;

	current_angle += current_rate;

	if (current_angle < FLIP_MID_ANGLE) {
		/* First half: accelerate rotation */
		if (current_rate < FLIP_MAX_RATE)
			current_rate += FLIP_DELTA_RATE;
		else
			max_rate_cnt++;
	} else {
		/* Second half: decelerate rotation */
		if (max_rate_cnt > 0) {
			max_rate_cnt--;
		} else {
			if (current_rate >= FLIP_DELTA_RATE &&
			    current_angle < 2 * FLIP_MID_ANGLE) {
				current_rate -= FLIP_DELTA_RATE;
			} else {
				flip_state = FLIP_STATE_FINISHED;
				return;
			}
		}
	}

	/* Apply rotation rate to the appropriate axis */
	switch (flip_dir) {
	case FLIP_DIR_FORWARD:
		sp->attitude.pitch = current_rate;
		sp->attitude.roll  = saved_roll;
		sp->attitude.yaw   = saved_yaw;
		break;
	case FLIP_DIR_BACK:
		sp->attitude.pitch = -current_rate;
		sp->attitude.roll  = saved_roll;
		sp->attitude.yaw   = saved_yaw;
		break;
	case FLIP_DIR_LEFT:
		sp->attitude.roll  = -current_rate;
		sp->attitude.pitch = saved_pitch;
		sp->attitude.yaw   = saved_yaw;
		break;
	case FLIP_DIR_RIGHT:
		sp->attitude.roll  = current_rate;
		sp->attitude.pitch = saved_pitch;
		sp->attitude.yaw   = saved_yaw;
		break;
	default:
		break;
	}
}

/*
 * State: FINISHED
 *
 * Flip rotation complete. Reset direction and prepare for
 * thrust recovery.
 */
static void state_finished(control_t *ctl)
{
	flip_dir  = FLIP_DIR_CENTER;
	ctl->flip_dir = FLIP_DIR_CENTER;

	recovery_cnt = 0;
	timeout_cnt  = 0;

	flip_state = FLIP_STATE_RECOVERY;
}

/*
 * State: RECOVERY
 *
 * Ramp thrust back up to stabilize the drone after the flip.
 */
static void state_recovery(setpoint_t *sp)
{
	if (recovery_cnt++ < FLIP_RECOVERY_TIME) {
		if (temp_thrust < flip_thrust_max)
			temp_thrust += 2 * (uint16_t)delta_thrust;
		sp->mode.z = MODE_DISABLE;
		sp->thrust = temp_thrust;
	} else {
		timeout_cnt = 0;
		flip_state  = FLIP_STATE_IDLE;
	}
}

/*
 * State: ERROR
 *
 * Flip timed out. Cut thrust briefly, then return to idle.
 * If the drone was in key-flight mode, restore altitude hold.
 */
static void state_error(setpoint_t *sp)
{
	flip_dir       = FLIP_DIR_CENTER;
	timeout_cnt    = 0;
	recovery_cnt   = 0;

	sp->mode.z = MODE_DISABLE;
	sp->thrust = 0;

	flip_state = FLIP_STATE_IDLE;

	if (commander_get_key_flight()) {
		sp->thrust = 0;
		sp->mode.z = MODE_ABS;
	}
}

void flip_check(setpoint_t *sp, control_t *ctl, const state_t *state)
{
	switch (flip_state) {
	case FLIP_STATE_IDLE:
		state_idle(ctl, state);
		break;
	case FLIP_STATE_SET:
		state_set(state);
		break;
	case FLIP_STATE_SPEED_UP:
		state_speed_up(sp, state);
		break;
	case FLIP_STATE_SLOW_DOWN:
		state_slow_down(sp);
		/* fall through to rotate if transitioned */
		if (flip_state == FLIP_STATE_ROTATE)
			state_rotate(sp, ctl);
		break;
	case FLIP_STATE_ROTATE:
		state_rotate(sp, ctl);
		break;
	case FLIP_STATE_FINISHED:
		state_finished(ctl);
		break;
	case FLIP_STATE_RECOVERY:
		state_recovery(sp);
		break;
	case FLIP_STATE_ERROR:
		state_error(sp);
		break;
	default:
		flip_state = FLIP_STATE_IDLE;
		break;
	}
}
