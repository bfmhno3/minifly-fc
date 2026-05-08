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

#include <math.h>
#include <string.h>

#include "control/attitude_estimator.h"
#include "comm/commander.h"

/* axis indices */
#define X 0
#define Y 1
#define Z 2

/* gravity in cm/s² */
#define GRAVITY_CMSS 980.0f

/* acceleration output limits, cm/s² */
#define ACC_LIMIT_NORMAL 1000.0f
#define ACC_LIMIT_ACRO 1800.0f

/* velocity output limits, cm/s */
#define VEL_LIMIT_NORMAL 130.0f
#define VEL_LIMIT_ACRO 500.0f

/* correction gains */
#define BARO_CORRECTION_W 0.35f
#define OPFLOW_POS_W 1.0f
#define OPFLOW_VEL_W 2.0f
#define ACC_BIAS_CORRECTION_W 0.01f

/* max acceptable accel bias correction magnitude, 0.25G */
#define ACC_BIAS_ACCEPTANCE (GRAVITY_CMSS * 0.25f)

/* acceleration deadband to suppress drift, cm/s² */
#define ACC_DEADBAND 4.0f

/* exponential LPF alpha values */
#define ACC_LPF_ALPHA 0.1f
#define RANGE_LPF_ALPHA 0.1f
#define FUSED_H_LPF_ALPHA 0.1f

/* laser ranger validity thresholds */
#define LASER_MAX_RANGE 200.0f
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

/**
 * @brief Internal state container for the INAV-style position estimator.
 *
 * @details
 * This struct stores the estimator's persistent state across update cycles:
 * - bias-compensated acceleration in earth frame
 * - integrated velocity/position states
 * - fused altitude references from barometer and laser range
 * - deferred reset flags to keep reset behavior deterministic inside update()
 *
 * Units:
 * - acceleration: cm/s^2
 * - velocity: cm/s
 * - position/height/range: cm
 * - baro_ground_asl: same ASL unit as sensor->baro.asl (project uses cm)
 */
typedef struct pos_estimator {
  float acc_bias[3]; // Estimated accelerometer bias in body frame.
  float
    acc_earth[3]; // Current earth-frame acceleration after bias removal/deadband.
  float acc_lpf[3];   // LPF-smoothed acceleration for output stabilization.
  float vel[3];       // Integrated velocity state.
  float pos[3];       // Integrated position state.
  float acc_deadband; // Deadband threshold to suppress near-zero accel drift.

  float baro_ground_asl; // Dynamic barometer ground reference (ASL baseline).
  float fused_height; // Instant fused height from baro + laser quality weighting.
  float fused_height_lpf; // LPF-smoothed fused height exposed to external users.
  float range_lpf;        // LPF-smoothed laser range used in height fusion.

  bool
    reset_height_pending; // Deferred Z-only reset, applied in update() for consistency.
  bool reset_all_pending; // Deferred full estimator reset, applied in update().
} pos_estimator_t;

static pos_estimator_t g_pes;

/* --- INAV filter core --- */

/** @brief  Dead-reckoning prediction: pos += vel*dt + 0.5*acc*dt^2 */
static void inav_predict(int axis, float dt, float acc)
{
  g_pes.pos[axis] += g_pes.vel[axis] * dt + acc * dt * dt * 0.5f;
  g_pes.vel[axis] += acc * dt;
}

/** @brief  Position correction: nudges both pos and vel proportionally to position error */
static void inav_correct_pos(int axis, float dt, float err, float w)
{
  float ewdt = err * w * dt;
  g_pes.pos[axis] += ewdt;
  g_pes.vel[axis] += w * ewdt;
}

/** @brief  Velocity-only correction (used for optical flow velocity feedback) */
static void inav_correct_vel(int axis, float dt, float err, float w)
{
  g_pes.vel[axis] += err * w * dt;
}

/* --- public API --- */

void position_estimator_init(void)
{
  memset(&g_pes, 0, sizeof(g_pes));
  g_pes.acc_deadband = ACC_DEADBAND;
  g_pes.reset_all_pending = true;
}

void position_estimator_update(state_t *state, const sensor_data_t *sensor,
                               const pos_estimator_ext_t *ext, float dt)
{
  bool has_opflow = ext && ext->valid;
  float weight = BARO_CORRECTION_W;

  /* ---- height fusion ---- */
  float rel_height = sensor->baro.asl - g_pes.baro_ground_asl;

  if (has_opflow && ext->laser_quality >= LASER_QUALITY_MIN) {
    g_pes.range_lpf =
      lpf_step(g_pes.range_lpf, ext->laser_range, RANGE_LPF_ALPHA);

    float quality = ext->laser_quality;
    weight = quality;
    g_pes.baro_ground_asl = sensor->baro.asl - g_pes.range_lpf;
    g_pes.fused_height =
      g_pes.range_lpf * quality + (1.0f - quality) * rel_height;
  } else {
    g_pes.fused_height = rel_height;
  }

  g_pes.fused_height_lpf =
    lpf_step(g_pes.fused_height_lpf, g_pes.fused_height, FUSED_H_LPF_ALPHA);

  /* ---- reset handling ---- */
  if (g_pes.reset_height_pending) {
    g_pes.reset_height_pending = false;
    weight = 0.95f;

    g_pes.baro_ground_asl = sensor->baro.asl;
    if (has_opflow && ext->laser_range < LASER_MAX_RANGE) {
      g_pes.baro_ground_asl -= ext->laser_range;
      g_pes.fused_height = ext->laser_range;
    }

    g_pes.pos[Z] = g_pes.fused_height;
  } else if (g_pes.reset_all_pending) {
    g_pes.reset_all_pending = false;

    g_pes.acc_lpf[Z] = 0.0f;
    g_pes.fused_height = 0.0f;
    g_pes.fused_height_lpf = 0.0f;

    g_pes.baro_ground_asl = sensor->baro.asl;
    if (has_opflow && ext->laser_range < LASER_MAX_RANGE) {
      g_pes.baro_ground_asl -= ext->laser_range;
      g_pes.fused_height = ext->laser_range;
    }

    g_pes.vel[Z] = 0.0f;
    g_pes.pos[Z] = g_pes.fused_height;
  }

  /* ---- earth-frame acceleration ---- */
  axis3f_t acc_bf;
  acc_bf.x = sensor->acc.x * GRAVITY_CMSS - g_pes.acc_bias[X];
  acc_bf.y = sensor->acc.y * GRAVITY_CMSS - g_pes.acc_bias[Y];
  acc_bf.z = sensor->acc.z * GRAVITY_CMSS - g_pes.acc_bias[Z];

  attitude_estimator_body_to_earth(&acc_bf);

  g_pes.acc_earth[X] = apply_deadband(acc_bf.x, g_pes.acc_deadband);
  g_pes.acc_earth[Y] = apply_deadband(acc_bf.y, g_pes.acc_deadband);
  g_pes.acc_earth[Z] = apply_deadband(acc_bf.z, g_pes.acc_deadband);

  for (int i = 0; i < 3; i++)
    g_pes.acc_lpf[i] =
      lpf_step(g_pes.acc_lpf[i], g_pes.acc_earth[i], ACC_LPF_ALPHA);

  /* ---- output acceleration (constrained) ---- */
  bool flight_land = commander_get_key_flight() || commander_get_key_land();
  float acc_lim = flight_land ? ACC_LIMIT_NORMAL : ACC_LIMIT_ACRO;

  state->acc.x = constrainf(g_pes.acc_lpf[X], -acc_lim, acc_lim);
  state->acc.y = constrainf(g_pes.acc_lpf[Y], -acc_lim, acc_lim);
  state->acc.z = constrainf(g_pes.acc_lpf[Z], -acc_lim, acc_lim);

  /* ---- Z-axis prediction + baro correction ---- */
  float err_pos_z = g_pes.fused_height - g_pes.pos[Z];

  inav_predict(Z, dt, g_pes.acc_earth[Z]);
  inav_correct_pos(Z, dt, err_pos_z, weight);

  /* ---- XY-axis prediction + optical flow correction ---- */
  if (has_opflow) {
    float op_res_x = ext->pos_sum[X] - g_pes.pos[X];
    float op_res_y = ext->pos_sum[Y] - g_pes.pos[Y];
    float op_res_vx = ext->vel_lpf[X] - g_pes.vel[X];
    float op_res_vy = ext->vel_lpf[Y] - g_pes.vel[Y];

    inav_predict(X, dt, g_pes.acc_earth[X]);
    inav_predict(Y, dt, g_pes.acc_earth[Y]);

    inav_correct_pos(X, dt, op_res_x, OPFLOW_POS_W);
    inav_correct_pos(Y, dt, op_res_y, OPFLOW_POS_W);
    inav_correct_vel(X, dt, op_res_vx, OPFLOW_VEL_W);
    inav_correct_vel(Y, dt, op_res_vy, OPFLOW_VEL_W);
  }

  /* ---- accel bias correction from baro error ---- */
  {
    axis3f_t bias_corr;
    memset(&bias_corr, 0, sizeof(bias_corr));
    bias_corr.z -= err_pos_z * sq(BARO_CORRECTION_W);

    float mag_sq = sq(bias_corr.x) + sq(bias_corr.y) + sq(bias_corr.z);
    if (mag_sq < sq(ACC_BIAS_ACCEPTANCE)) {
      attitude_estimator_earth_to_body(&bias_corr);

      g_pes.acc_bias[X] += bias_corr.x * ACC_BIAS_CORRECTION_W * dt;
      g_pes.acc_bias[Y] += bias_corr.y * ACC_BIAS_CORRECTION_W * dt;
      g_pes.acc_bias[Z] += bias_corr.z * ACC_BIAS_CORRECTION_W * dt;
    }
  }

  /* ---- output velocity (constrained) ---- */
  float vel_lim = flight_land ? VEL_LIMIT_NORMAL : VEL_LIMIT_ACRO;

  state->velocity.x = constrainf(g_pes.vel[X], -vel_lim, vel_lim);
  state->velocity.y = constrainf(g_pes.vel[Y], -vel_lim, vel_lim);
  state->velocity.z = constrainf(g_pes.vel[Z], -vel_lim, vel_lim);

  /* ---- output position ---- */
  state->position.x = g_pes.pos[X];
  state->position.y = g_pes.pos[Y];
  state->position.z = g_pes.pos[Z];
}

void estimator_reset_height(void)
{
  g_pes.reset_height_pending = true;
}

void estimator_reset_all(void)
{
  g_pes.reset_all_pending = true;
}

float position_estimator_get_fused_height(void)
{
  return g_pes.fused_height_lpf;
}
