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

#define DEG2RAD 0.017453293f  /* pi / 180 */
#define RAD2DEG 57.29578f    /* 180 / pi */

#define DEFAULT_KP        0.4f    /* proportional gain for gyro correction */
#define DEFAULT_KI        0.001f  /* integral gain for gyro bias drift */
#define CALIB_SAMPLES     350U    /* number of samples for gravity calibration */
#define CALIB_THRESHOLD   0.015f  /* max acc variation (G) to accept calibration */
#define CALIB_Z_MIN_INIT  1.5f   /* initial high bound for min search (G) */
#define CALIB_Z_MAX_INIT  0.5f   /* initial low bound for max search (G) */

typedef struct {
	float q0, q1, q2, q3;
	float r_mat[3][3];
	float ex_int, ey_int, ez_int;
	float kp;
	float ki;
	float base_acc_z;
	float calib_z_min;
	float calib_z_max;
	float calib_z_sum;
	uint32_t calib_count;
	bool calibrated;
} attitude_estimator_t;

static attitude_estimator_t est;

/**
 * @brief  Rebuild the 3x3 rotation matrix from the current quaternion
 *
 * Uses the standard quaternion-to-DCM formula.  Called after every
 * quaternion update to keep r_mat in sync.
 */
static void compute_rotation_matrix(void)
{
	float q1q1 = est.q1 * est.q1;
	float q2q2 = est.q2 * est.q2;
	float q3q3 = est.q3 * est.q3;

	float q0q1 = est.q0 * est.q1;
	float q0q2 = est.q0 * est.q2;
	float q0q3 = est.q0 * est.q3;
	float q1q2 = est.q1 * est.q2;
	float q1q3 = est.q1 * est.q3;
	float q2q3 = est.q2 * est.q3;

	est.r_mat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
	est.r_mat[0][1] = 2.0f * (q1q2 - q0q3);
	est.r_mat[0][2] = 2.0f * (q1q3 + q0q2);

	est.r_mat[1][0] = 2.0f * (q1q2 + q0q3);
	est.r_mat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
	est.r_mat[1][2] = 2.0f * (q2q3 - q0q1);

	est.r_mat[2][0] = 2.0f * (q1q3 - q0q2);
	est.r_mat[2][1] = 2.0f * (q2q3 + q0q1);
	est.r_mat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
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
	est.calib_z_sum += earth_z;
	if (earth_z < est.calib_z_min)
		est.calib_z_min = earth_z;
	if (earth_z > est.calib_z_max)
		est.calib_z_max = earth_z;

	if (++est.calib_count < CALIB_SAMPLES)
		return;

	float variation = est.calib_z_max - est.calib_z_min;
	if (variation < CALIB_THRESHOLD) {
		est.base_acc_z = est.calib_z_sum / (float)CALIB_SAMPLES;
		est.calibrated = true;
	}

	est.calib_count = 0;
	est.calib_z_sum = 0.0f;
	est.calib_z_min = CALIB_Z_MIN_INIT;
	est.calib_z_max = CALIB_Z_MAX_INIT;
}

void attitude_estimator_init(void)
{
	est.q0 = 1.0f;
	est.q1 = 0.0f;
	est.q2 = 0.0f;
	est.q3 = 0.0f;

	est.ex_int = 0.0f;
	est.ey_int = 0.0f;
	est.ez_int = 0.0f;

	est.kp = DEFAULT_KP;
	est.ki = DEFAULT_KI;

	est.base_acc_z = 1.0f;
	est.calib_z_min = CALIB_Z_MIN_INIT;
	est.calib_z_max = CALIB_Z_MAX_INIT;
	est.calib_z_sum = 0.0f;
	est.calib_count = 0;
	est.calibrated = false;

	compute_rotation_matrix();
}

void attitude_estimator_update(const Axis3f *acc, const Axis3f *gyro, float dt)
{
	Axis3f gyro_rad;

	gyro_rad.x = gyro->x * DEG2RAD;
	gyro_rad.y = gyro->y * DEG2RAD;
	gyro_rad.z = gyro->z * DEG2RAD;

	if (acc->x != 0.0f || acc->y != 0.0f || acc->z != 0.0f) {
		float norm = 1.0f / sqrtf(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
		float ax = acc->x * norm;
		float ay = acc->y * norm;
		float az = acc->z * norm;

		float ex = (ay * est.r_mat[2][2] - az * est.r_mat[2][1]);
		float ey = (az * est.r_mat[2][0] - ax * est.r_mat[2][2]);
		float ez = (ax * est.r_mat[2][1] - ay * est.r_mat[2][0]);

		est.ex_int += est.ki * ex * dt;
		est.ey_int += est.ki * ey * dt;
		est.ez_int += est.ki * ez * dt;

		gyro_rad.x += est.kp * ex + est.ex_int;
		gyro_rad.y += est.kp * ey + est.ey_int;
		gyro_rad.z += est.kp * ez + est.ez_int;
	}

	/* First-order quaternion integration: dq/dt = 0.5 * q * omega */
	{
		float half_t = 0.5f * dt;
		float q0_last = est.q0;
		float q1_last = est.q1;
		float q2_last = est.q2;
		float q3_last = est.q3;

		est.q0 += (-q1_last * gyro_rad.x - q2_last * gyro_rad.y - q3_last * gyro_rad.z) * half_t;
		est.q1 += ( q0_last * gyro_rad.x + q2_last * gyro_rad.z - q3_last * gyro_rad.y) * half_t;
		est.q2 += ( q0_last * gyro_rad.y - q1_last * gyro_rad.z + q3_last * gyro_rad.x) * half_t;
		est.q3 += ( q0_last * gyro_rad.z + q1_last * gyro_rad.y - q2_last * gyro_rad.x) * half_t;
	}

	{
		float qnorm = 1.0f / sqrtf(est.q0 * est.q0 + est.q1 * est.q1 + est.q2 * est.q2 + est.q3 * est.q3);
		est.q0 *= qnorm;
		est.q1 *= qnorm;
		est.q2 *= qnorm;
		est.q3 *= qnorm;
	}

	compute_rotation_matrix();

	if (!est.calibrated) {
		float earth_z = acc->x * est.r_mat[2][0]
			      + acc->y * est.r_mat[2][1]
			      + acc->z * est.r_mat[2][2];
		run_gravity_calibration(earth_z);
	}
}

void attitude_estimator_get_attitude(attitude_t *attitude)
{
	attitude->roll  = atan2f(est.r_mat[2][1], est.r_mat[2][2]) * RAD2DEG;
	attitude->pitch = -asinf(est.r_mat[2][0]) * RAD2DEG;
	attitude->yaw   = atan2f(est.r_mat[1][0], est.r_mat[0][0]) * RAD2DEG;
	attitude->timestamp = 0;
}

void attitude_estimator_get_quaternion(quaternion_t *quat)
{
	quat->q0 = est.q0;
	quat->q1 = est.q1;
	quat->q2 = est.q2;
	quat->q3 = est.q3;
	quat->timestamp = 0;
}

bool attitude_estimator_is_calibrated(void)
{
	return est.calibrated;
}

void attitude_estimator_body_to_earth(Axis3f *v)
{
	float x = est.r_mat[0][0] * v->x + est.r_mat[0][1] * v->y + est.r_mat[0][2] * v->z;
	float y = est.r_mat[1][0] * v->x + est.r_mat[1][1] * v->y + est.r_mat[1][2] * v->z;
	float z = est.r_mat[2][0] * v->x + est.r_mat[2][1] * v->y + est.r_mat[2][2] * v->z;

	v->x = x;
	v->y = y;
	v->z = z;
}

void attitude_estimator_earth_to_body(Axis3f *v)
{
	float x = est.r_mat[0][0] * v->x + est.r_mat[1][0] * v->y + est.r_mat[2][0] * v->z;
	float y = est.r_mat[0][1] * v->x + est.r_mat[1][1] * v->y + est.r_mat[2][1] * v->z;
	float z = est.r_mat[0][2] * v->x + est.r_mat[1][2] * v->y + est.r_mat[2][2] * v->z;

	v->x = x;
	v->y = y;
	v->z = z;
}
