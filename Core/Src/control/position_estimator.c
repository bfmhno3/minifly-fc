// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  INAV-style position and velocity estimator
 *
 * @details
 * Predicts position/velocity from earth-frame acceleration, then corrects
 * using barometer (Z-axis) and optical flow (XY-axis) measurements.
 * Accelerometer bias is slowly estimated from the barometer residual.
 * All distances in cm, velocities in cm/s, accelerations in cm/s^2.
 */

#include "control/position_estimator.h"
#include "control/attitude_estimator.h"
#include "comm/commander.h"
#include <math.h>
#include <string.h>

/* axis indices */
#define X 0
#define Y 1
#define Z 2

/* gravity in cm/s² */
#define GRAVITY_CMSS 980.0f

/* acceleration output limits, cm/s² */
#define ACC_LIMIT_NORMAL 1000.0f
#define ACC_LIMIT_ACRO   1800.0f

/* velocity output limits, cm/s */
#define VEL_LIMIT_NORMAL 130.0f
#define VEL_LIMIT_ACRO   500.0f

/* correction gains */
#define BARO_CORRECTION_W       0.35f
#define OPFLOW_POS_W            1.0f
#define OPFLOW_VEL_W            2.0f
#define ACC_BIAS_CORRECTION_W   0.01f

/* max acceptable accel bias correction magnitude, 0.25G */
#define ACC_BIAS_ACCEPTANCE (GRAVITY_CMSS * 0.25f)

/* acceleration deadband to suppress drift, cm/s² */
#define ACC_DEADBAND 4.0f

/* exponential LPF alpha values */
#define ACC_LPF_ALPHA      0.1f
#define RANGE_LPF_ALPHA    0.1f
#define FUSED_H_LPF_ALPHA  0.1f

/* laser ranger validity thresholds */
#define LASER_MAX_RANGE   200.0f
#define LASER_QUALITY_MIN 0.3f

/* --- helpers --- */

/** @brief  Clamp val to [min, max] */
static float constrainf(float val, float min, float max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;
	return val;
}

/** @brief  Return 0 when |val| < deadband, otherwise pass through */
static float apply_deadband(float val, float deadband)
{
	if (fabsf(val) < deadband)
		return 0.0f;
	return val;
}

static float sq(float x)
{
	return x * x;
}

/** @brief  Single-pole IIR low-pass filter step: cur + (sample - cur) * alpha */
static float lpf_step(float cur, float sample, float alpha)
{
	return cur + (sample - cur) * alpha;
}

/* --- private state --- */

typedef struct {
	float acc_bias[3];
	float acc_earth[3];
	float acc_lpf[3];
	float vel[3];
	float pos[3];
	float acc_deadband;

	float baro_ground_asl;
	float fused_height;
	float fused_height_lpf;
	float range_lpf;

	bool reset_height_pending;
	bool reset_all_pending;
} pos_estimator_t;

static pos_estimator_t pes;

/* --- INAV filter core --- */

/** @brief  Dead-reckoning prediction: pos += vel*dt + 0.5*acc*dt^2 */
static void inav_predict(int axis, float dt, float acc)
{
	pes.pos[axis] += pes.vel[axis] * dt + acc * dt * dt * 0.5f;
	pes.vel[axis] += acc * dt;
}

/** @brief  Position correction: nudges both pos and vel proportionally to position error */
static void inav_correct_pos(int axis, float dt, float err, float w)
{
	float ewdt = err * w * dt;
	pes.pos[axis] += ewdt;
	pes.vel[axis] += w * ewdt;
}

/** @brief  Velocity-only correction (used for optical flow velocity feedback) */
static void inav_correct_vel(int axis, float dt, float err, float w)
{
	pes.vel[axis] += err * w * dt;
}

/* --- public API --- */

void position_estimator_init(void)
{
	memset(&pes, 0, sizeof(pes));
	pes.acc_deadband = ACC_DEADBAND;
	pes.reset_all_pending = true;
}

void position_estimator_update(state_t *state, const sensor_data_t *sensor,
	const pos_estimator_ext_t *ext, float dt)
{
	bool has_opflow = ext && ext->valid;
	float weight = BARO_CORRECTION_W;

	/* ---- height fusion ---- */
	float rel_height = sensor->baro.asl - pes.baro_ground_asl;

	if (has_opflow && ext->laser_quality >= LASER_QUALITY_MIN) {
		pes.range_lpf = lpf_step(pes.range_lpf, ext->laser_range,
			RANGE_LPF_ALPHA);

		float quality = ext->laser_quality;
		weight = quality;
		pes.baro_ground_asl = sensor->baro.asl - pes.range_lpf;
		pes.fused_height = pes.range_lpf * quality +
			(1.0f - quality) * rel_height;
	} else {
		pes.fused_height = rel_height;
	}

	pes.fused_height_lpf = lpf_step(pes.fused_height_lpf,
		pes.fused_height, FUSED_H_LPF_ALPHA);

	/* ---- reset handling ---- */
	if (pes.reset_height_pending) {
		pes.reset_height_pending = false;
		weight = 0.95f;

		pes.baro_ground_asl = sensor->baro.asl;
		if (has_opflow && ext->laser_range < LASER_MAX_RANGE) {
			pes.baro_ground_asl -= ext->laser_range;
			pes.fused_height = ext->laser_range;
		}

		pes.pos[Z] = pes.fused_height;
	} else if (pes.reset_all_pending) {
		pes.reset_all_pending = false;

		pes.acc_lpf[Z] = 0.0f;
		pes.fused_height = 0.0f;
		pes.fused_height_lpf = 0.0f;

		pes.baro_ground_asl = sensor->baro.asl;
		if (has_opflow && ext->laser_range < LASER_MAX_RANGE) {
			pes.baro_ground_asl -= ext->laser_range;
			pes.fused_height = ext->laser_range;
		}

		pes.vel[Z] = 0.0f;
		pes.pos[Z] = pes.fused_height;
	}

	/* ---- earth-frame acceleration ---- */
	Axis3f acc_bf;
	acc_bf.x = sensor->acc.x * GRAVITY_CMSS - pes.acc_bias[X];
	acc_bf.y = sensor->acc.y * GRAVITY_CMSS - pes.acc_bias[Y];
	acc_bf.z = sensor->acc.z * GRAVITY_CMSS - pes.acc_bias[Z];

	attitude_estimator_body_to_earth(&acc_bf);

	pes.acc_earth[X] = apply_deadband(acc_bf.x, pes.acc_deadband);
	pes.acc_earth[Y] = apply_deadband(acc_bf.y, pes.acc_deadband);
	pes.acc_earth[Z] = apply_deadband(acc_bf.z, pes.acc_deadband);

	for (int i = 0; i < 3; i++)
		pes.acc_lpf[i] = lpf_step(pes.acc_lpf[i], pes.acc_earth[i],
			ACC_LPF_ALPHA);

	/* ---- output acceleration (constrained) ---- */
	bool flight_land = commander_get_key_flight()
		|| commander_get_key_land();
	float acc_lim = flight_land ? ACC_LIMIT_NORMAL : ACC_LIMIT_ACRO;

	state->acc.x = constrainf(pes.acc_lpf[X], -acc_lim, acc_lim);
	state->acc.y = constrainf(pes.acc_lpf[Y], -acc_lim, acc_lim);
	state->acc.z = constrainf(pes.acc_lpf[Z], -acc_lim, acc_lim);

	/* ---- Z-axis prediction + baro correction ---- */
	float err_pos_z = pes.fused_height - pes.pos[Z];

	inav_predict(Z, dt, pes.acc_earth[Z]);
	inav_correct_pos(Z, dt, err_pos_z, weight);

	/* ---- XY-axis prediction + optical flow correction ---- */
	if (has_opflow) {
		float op_res_x = ext->pos_sum[X] - pes.pos[X];
		float op_res_y = ext->pos_sum[Y] - pes.pos[Y];
		float op_res_vx = ext->vel_lpf[X] - pes.vel[X];
		float op_res_vy = ext->vel_lpf[Y] - pes.vel[Y];

		inav_predict(X, dt, pes.acc_earth[X]);
		inav_predict(Y, dt, pes.acc_earth[Y]);

		inav_correct_pos(X, dt, op_res_x, OPFLOW_POS_W);
		inav_correct_pos(Y, dt, op_res_y, OPFLOW_POS_W);
		inav_correct_vel(X, dt, op_res_vx, OPFLOW_VEL_W);
		inav_correct_vel(Y, dt, op_res_vy, OPFLOW_VEL_W);
	}

	/* ---- accel bias correction from baro error ---- */
	{
		Axis3f bias_corr;
		memset(&bias_corr, 0, sizeof(bias_corr));
		bias_corr.z -= err_pos_z * sq(BARO_CORRECTION_W);

		float mag_sq = sq(bias_corr.x) + sq(bias_corr.y)
			+ sq(bias_corr.z);
		if (mag_sq < sq(ACC_BIAS_ACCEPTANCE)) {
			attitude_estimator_earth_to_body(&bias_corr);

			pes.acc_bias[X] += bias_corr.x * ACC_BIAS_CORRECTION_W * dt;
			pes.acc_bias[Y] += bias_corr.y * ACC_BIAS_CORRECTION_W * dt;
			pes.acc_bias[Z] += bias_corr.z * ACC_BIAS_CORRECTION_W * dt;
		}
	}

	/* ---- output velocity (constrained) ---- */
	float vel_lim = flight_land ? VEL_LIMIT_NORMAL : VEL_LIMIT_ACRO;

	state->velocity.x = constrainf(pes.vel[X], -vel_lim, vel_lim);
	state->velocity.y = constrainf(pes.vel[Y], -vel_lim, vel_lim);
	state->velocity.z = constrainf(pes.vel[Z], -vel_lim, vel_lim);

	/* ---- output position ---- */
	state->position.x = pes.pos[X];
	state->position.y = pes.pos[Y];
	state->position.z = pes.pos[Z];
}

void estimator_reset_height(void)
{
	pes.reset_height_pending = true;
}

void estimator_reset_all(void)
{
	pes.reset_all_pending = true;
}

float position_estimator_get_fused_height(void)
{
	return pes.fused_height_lpf;
}
