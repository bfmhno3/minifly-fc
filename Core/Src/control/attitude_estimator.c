// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Mahony complementary filter for attitude estimation
 *
 * @details
 * Maintains a unit quaternion updated at 250 Hz.  Gyro drift is corrected
 * by the gravity vector observed through the accelerometer.  A one-shot
 * gravity calibration stage runs on startup to learn the static Z-axis
 * accelerometer bias.
 */

#include "control/attitude_estimator.h"

#include <math.h>

#define DEG2RAD 0.017453293f /* pi / 180 */
#define RAD2DEG 57.29578f    /* 180 / pi */

#define DEFAULT_KP 0.4f        /* proportional gain for gyro correction */
#define DEFAULT_KI 0.001f      /* integral gain for gyro bias drift */
#define CALIB_SAMPLES 350U     /* number of samples for gravity calibration */
#define CALIB_THRESHOLD 0.015f /* max acc variation (G) to accept calibration */
#define CALIB_Z_MIN_INIT 1.5f  /* initial high bound for min search (G) */
#define CALIB_Z_MAX_INIT 0.5f  /* initial low bound for max search (G) */

/**
 * @brief Internal state container for the Mahony attitude estimator.
 *
 * @details
 * Stores the full filter state across update cycles: orientation quaternion,
 * cached rotation matrix, PI correction integrators, and startup gravity-bias
 * calibration accumulators. Kept module-private to preserve estimator
 * consistency and avoid external partial writes.
 */
typedef struct attitude_estimator {
  float q0, q1, q2, q3; // Current unit quaternion (body -> earth).
  float r_mat[3][3];    // Rotation matrix rebuilt from quaternion each update.

  float ex_int, ey_int, ez_int; // Integral terms for gyro bias compensation.
  float kp;          // Proportional gain for accelerometer correction.
  float ki;          // Integral gain for long-term drift rejection.

  float base_acc_z;  // Learned static earth-frame Z acceleration baseline (G).
  float calib_z_min; // Min sample in current calibration window (G).
  float calib_z_max; // Max sample in current calibration window (G).
  float calib_z_sum; // Sum of samples in current calibration window (G).
  uint32_t calib_count; // Number of collected samples in current window.
  bool calibrated;      // True after a stable gravity baseline is accepted.
} attitude_estimator_t;

static attitude_estimator_t g_att;

/**
 * @brief  Rebuild the 3x3 rotation matrix from the current quaternion
 *
 * Uses the standard quaternion-to-DCM formula.  Called after every
 * quaternion update to keep r_mat in sync.
 */
static void compute_rotation_matrix(void)
{
  float q1q1 = g_att.q1 * g_att.q1;
  float q2q2 = g_att.q2 * g_att.q2;
  float q3q3 = g_att.q3 * g_att.q3;

  float q0q1 = g_att.q0 * g_att.q1;
  float q0q2 = g_att.q0 * g_att.q2;
  float q0q3 = g_att.q0 * g_att.q3;
  float q1q2 = g_att.q1 * g_att.q2;
  float q1q3 = g_att.q1 * g_att.q3;
  float q2q3 = g_att.q2 * g_att.q3;

  g_att.r_mat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
  g_att.r_mat[0][1] = 2.0f * (q1q2 - q0q3);
  g_att.r_mat[0][2] = 2.0f * (q1q3 + q0q2);

  g_att.r_mat[1][0] = 2.0f * (q1q2 + q0q3);
  g_att.r_mat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
  g_att.r_mat[1][2] = 2.0f * (q2q3 - q0q1);

  g_att.r_mat[2][0] = 2.0f * (q1q3 - q0q2);
  g_att.r_mat[2][1] = 2.0f * (q2q3 + q0q1);
  g_att.r_mat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

/**
 * @brief  Accumulate earth-frame Z acceleration to find the static gravity offset
 *
 * Runs for CALIB_SAMPLES iterations.  If the variation between min and max
 * readings stays below CALIB_THRESHOLD, the mean is accepted as base_acc_z.
 * Otherwise the window resets and retries.
 *
 * @param[in] earth_z  Earth-frame Z-axis acceleration in G
 */
static void run_gravity_calibration(float earth_z)
{
  g_att.calib_z_sum += earth_z;
  if (earth_z < g_att.calib_z_min)
    g_att.calib_z_min = earth_z;
  if (earth_z > g_att.calib_z_max)
    g_att.calib_z_max = earth_z;

  if (++g_att.calib_count < CALIB_SAMPLES)
    return;

  float variation = g_att.calib_z_max - g_att.calib_z_min;
  if (variation < CALIB_THRESHOLD) {
    g_att.base_acc_z = g_att.calib_z_sum / (float)CALIB_SAMPLES;
    g_att.calibrated = true;
  }

  g_att.calib_count = 0;
  g_att.calib_z_sum = 0.0f;
  g_att.calib_z_min = CALIB_Z_MIN_INIT;
  g_att.calib_z_max = CALIB_Z_MAX_INIT;
}

void attitude_estimator_init(void)
{
  g_att.q0 = 1.0f;
  g_att.q1 = 0.0f;
  g_att.q2 = 0.0f;
  g_att.q3 = 0.0f;

  g_att.ex_int = 0.0f;
  g_att.ey_int = 0.0f;
  g_att.ez_int = 0.0f;

  g_att.kp = DEFAULT_KP;
  g_att.ki = DEFAULT_KI;

  g_att.base_acc_z = 1.0f;
  g_att.calib_z_min = CALIB_Z_MIN_INIT;
  g_att.calib_z_max = CALIB_Z_MAX_INIT;
  g_att.calib_z_sum = 0.0f;
  g_att.calib_count = 0;
  g_att.calibrated = false;

  compute_rotation_matrix();
}

void attitude_estimator_update(const axis3f_t *acc, const axis3f_t *gyro,
                               float dt)
{
  axis3f_t gyro_rad;

  gyro_rad.x = gyro->x * DEG2RAD;
  gyro_rad.y = gyro->y * DEG2RAD;
  gyro_rad.z = gyro->z * DEG2RAD;

  if (acc->x != 0.0f || acc->y != 0.0f || acc->z != 0.0f) {
    float norm =
      1.0f / sqrtf(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
    float ax = acc->x * norm;
    float ay = acc->y * norm;
    float az = acc->z * norm;

    float ex = (ay * g_att.r_mat[2][2] - az * g_att.r_mat[2][1]);
    float ey = (az * g_att.r_mat[2][0] - ax * g_att.r_mat[2][2]);
    float ez = (ax * g_att.r_mat[2][1] - ay * g_att.r_mat[2][0]);

    g_att.ex_int += g_att.ki * ex * dt;
    g_att.ey_int += g_att.ki * ey * dt;
    g_att.ez_int += g_att.ki * ez * dt;

    gyro_rad.x += g_att.kp * ex + g_att.ex_int;
    gyro_rad.y += g_att.kp * ey + g_att.ey_int;
    gyro_rad.z += g_att.kp * ez + g_att.ez_int;
  }

  /* First-order quaternion integration: dq/dt = 0.5 * q * omega */
  {
    float half_t = 0.5f * dt;
    float q0_last = g_att.q0;
    float q1_last = g_att.q1;
    float q2_last = g_att.q2;
    float q3_last = g_att.q3;

    g_att.q0 +=
      (-q1_last * gyro_rad.x - q2_last * gyro_rad.y - q3_last * gyro_rad.z) *
      half_t;
    g_att.q1 +=
      (q0_last * gyro_rad.x + q2_last * gyro_rad.z - q3_last * gyro_rad.y) *
      half_t;
    g_att.q2 +=
      (q0_last * gyro_rad.y - q1_last * gyro_rad.z + q3_last * gyro_rad.x) *
      half_t;
    g_att.q3 +=
      (q0_last * gyro_rad.z + q1_last * gyro_rad.y - q2_last * gyro_rad.x) *
      half_t;
  }

  {
    float qnorm = 1.0f / sqrtf(g_att.q0 * g_att.q0 + g_att.q1 * g_att.q1 +
                               g_att.q2 * g_att.q2 + g_att.q3 * g_att.q3);
    g_att.q0 *= qnorm;
    g_att.q1 *= qnorm;
    g_att.q2 *= qnorm;
    g_att.q3 *= qnorm;
  }

  compute_rotation_matrix();

  if (!g_att.calibrated) {
    float earth_z = acc->x * g_att.r_mat[2][0] + acc->y * g_att.r_mat[2][1] +
                    acc->z * g_att.r_mat[2][2];
    run_gravity_calibration(earth_z);
  }
}

void attitude_estimator_get_attitude(attitude_t *attitude)
{
  attitude->roll = atan2f(g_att.r_mat[2][1], g_att.r_mat[2][2]) * RAD2DEG;
  attitude->pitch = -asinf(g_att.r_mat[2][0]) * RAD2DEG;
  attitude->yaw = atan2f(g_att.r_mat[1][0], g_att.r_mat[0][0]) * RAD2DEG;
  attitude->timestamp = 0;
}

void attitude_estimator_get_quaternion(quaternion_t *quat)
{
  quat->q0 = g_att.q0;
  quat->q1 = g_att.q1;
  quat->q2 = g_att.q2;
  quat->q3 = g_att.q3;
  quat->timestamp = 0;
}

bool attitude_estimator_is_calibrated(void)
{
  return g_att.calibrated;
}

void attitude_estimator_body_to_earth(axis3f_t *v)
{
  float x = g_att.r_mat[0][0] * v->x + g_att.r_mat[0][1] * v->y +
            g_att.r_mat[0][2] * v->z;
  float y = g_att.r_mat[1][0] * v->x + g_att.r_mat[1][1] * v->y +
            g_att.r_mat[1][2] * v->z;
  float z = g_att.r_mat[2][0] * v->x + g_att.r_mat[2][1] * v->y +
            g_att.r_mat[2][2] * v->z;

  v->x = x;
  v->y = y;
  v->z = z;
}

void attitude_estimator_earth_to_body(axis3f_t *v)
{
  float x = g_att.r_mat[0][0] * v->x + g_att.r_mat[1][0] * v->y +
            g_att.r_mat[2][0] * v->z;
  float y = g_att.r_mat[0][1] * v->x + g_att.r_mat[1][1] * v->y +
            g_att.r_mat[2][1] * v->z;
  float z = g_att.r_mat[0][2] * v->x + g_att.r_mat[1][2] * v->y +
            g_att.r_mat[2][2] * v->z;

  v->x = x;
  v->y = y;
  v->z = z;
}
