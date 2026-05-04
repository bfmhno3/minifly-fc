// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Cascaded PID coordinator implementation
 *
 * @details
 * Orchestrates position -> angle -> rate PID loops at their respective
 * sub-rates.  When thrust falls below a threshold, all PIDs are reset
 * and the yaw reference is snapped to the current heading to prevent
 * integral windup on the ground.
 */

#include "control/state_control.h"
#include "control/attitude_pid.h"
#include "control/position_pid.h"
#include "services/config_service.h"
#include <math.h>
#include <limits.h>

/* Sub-loop rates (Hz) — must divide evenly into the 500 Hz master tick */
#define POSITION_PID_RATE  250
#define ANGLE_PID_RATE     250
#define RATE_PID_RATE      500

#define POSITION_PID_DT    (1.0f / POSITION_PID_RATE)
#define ANGLE_PID_DT       (1.0f / ANGLE_PID_RATE)
#define RATE_PID_DT        (1.0f / RATE_PID_RATE)

#define THRUST_OFF_THRESHOLD  5.0f   /* below this, motors are considered off */
#define PID_RESET_HOLD_TICKS  1500   /* ticks before persisting config on ground */

static float actual_thrust;
static attitude_t att_desired;
static Axis3f rate_desired;
static uint16_t pid_off_cnt;

static void wrap_angle_180(float *angle)
{
	*angle = fmodf(*angle + 180.0f, 360.0f) - 180.0f;
}

static int16_t clamp_int16(float v)
{
	if (v > (float)INT16_MAX)
		return INT16_MAX;
	if (v < (float)(-INT16_MAX))
		return -INT16_MAX;
	return (int16_t)v;
}

void state_control_init(void)
{
	attitude_pid_init(RATE_PID_DT, ANGLE_PID_DT);
	position_pid_init(ANGLE_PID_DT, POSITION_PID_DT);

	actual_thrust = 0.0f;
	pid_off_cnt = 0;

	att_desired.roll = 0.0f;
	att_desired.pitch = 0.0f;
	att_desired.yaw = 0.0f;

	rate_desired.x = 0.0f;
	rate_desired.y = 0.0f;
	rate_desired.z = 0.0f;
}

bool state_control_test(void)
{
	return true;
}

void state_control_run(control_t *out, const sensor_data_t *sensor,
		       const state_t *state, const setpoint_t *sp,
		       uint32_t tick)
{
	Axis3f control_out = { .x = 0.0f, .y = 0.0f, .z = 0.0f };

	/* position pid: outer loop at 250 hz */
	if (RATE_DO_EXECUTE(POSITION_PID_RATE, tick)) {
		if (sp->mode.x != MODE_DISABLE ||
		    sp->mode.y != MODE_DISABLE ||
		    sp->mode.z != MODE_DISABLE) {
			position_pid_run((setpoint_t *)sp, state,
					 &att_desired, &actual_thrust);
		}
	}

	/* angle pid: mid loop at 250 hz */
	if (RATE_DO_EXECUTE(ANGLE_PID_RATE, tick)) {
		attitude_t att_with_trim;

		if (sp->mode.z == MODE_DISABLE)
			actual_thrust = sp->thrust;

		if (sp->mode.x == MODE_DISABLE ||
		    sp->mode.y == MODE_DISABLE) {
			att_desired.roll = sp->attitude.roll;
			att_desired.pitch = sp->attitude.pitch;
		}

		/* Accumulate yaw rate command into absolute yaw angle.
		 * Skip during flips so the flip state machine owns yaw. */
		if (out->flip_dir == FLIP_DIR_CENTER) {
			att_desired.yaw += sp->attitude.yaw / ANGLE_PID_RATE;
			wrap_angle_180(&att_desired.yaw);
		}

		att_with_trim.roll = att_desired.roll + config_service_get()->trimR;
		att_with_trim.pitch = att_desired.pitch + config_service_get()->trimP;
		att_with_trim.yaw = att_desired.yaw;

		attitude_pid_run_angle(&state->attitude, &att_with_trim,
				       &rate_desired);
	}

	/* rate pid: inner loop at 500 hz */
	if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
		if (sp->mode.roll == MODE_VELOCITY)
			rate_desired.roll = sp->attitudeRate.roll;
		if (sp->mode.pitch == MODE_VELOCITY)
			rate_desired.pitch = sp->attitudeRate.pitch;

		attitude_pid_run_rate(&sensor->gyro, &rate_desired,
				      &control_out);

		out->roll = clamp_int16(control_out.x);
		out->pitch = clamp_int16(control_out.y);
		out->yaw = clamp_int16(control_out.z);
	}

	out->thrust = actual_thrust;

	/* Motors off: reset all PID state to prevent integral windup on the
	 * ground, and snap yaw reference to current heading so the next takeoff
	 * doesn't chase a stale heading. */
	if (out->thrust < THRUST_OFF_THRESHOLD) {
		out->roll = 0;
		out->pitch = 0;
		out->yaw = 0;

		attitude_pid_reset();
		position_pid_reset();
		att_desired.yaw = state->attitude.yaw;

		if (pid_off_cnt++ > PID_RESET_HOLD_TICKS) {
			pid_off_cnt = 0;
			config_service_mark_dirty();
		}
	} else {
		pid_off_cnt = 0;
	}
}
