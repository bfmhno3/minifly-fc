#include <string.h>
#include <math.h>

#include "platform/maths.h"

/*
 * fast trig: order-9 Chebyshev approximation
 *
 * Max sin error < 2.6e-6 over [-pi, pi].
 * Atan2 max absolute error < 0.000027 degrees.
 * Acos absolute error <= 6.7e-5.
 */

#ifndef MATHS_NO_FAST_TRIG

#define SIN_COEF3  (-1.666665710e-1f)
#define SIN_COEF5  ( 8.333017292e-3f)
#define SIN_COEF7  (-1.980661520e-4f)
#define SIN_COEF9  ( 2.600054768e-6f)

float maths_sin(float x)
{
	int32_t xint = (int32_t)x;

	if (xint < -32 || xint > 32)
		return 0.0f;

	while (x >  MATHS_PI_F) x -= 2.0f * MATHS_PI_F;
	while (x < -MATHS_PI_F) x += 2.0f * MATHS_PI_F;

	if (x >  0.5f * MATHS_PI_F)
		x =  MATHS_PI_F - x;
	else if (x < -0.5f * MATHS_PI_F)
		x = -MATHS_PI_F - x;

	float x2 = x * x;
	return x + x * x2 * (SIN_COEF3 + x2 * (SIN_COEF5 +
	       x2 * (SIN_COEF7 + x2 * SIN_COEF9)));
}

float maths_cos(float x)
{
	return maths_sin(x + 0.5f * MATHS_PI_F);
}

float maths_atan2(float y, float x)
{
#define ATAN_COEF1  3.14551665884836e-07f
#define ATAN_COEF2  0.99997356613987f
#define ATAN_COEF3  0.14744007058297684f
#define ATAN_COEF4  0.3099814292351353f
#define ATAN_COEF5  0.05030176425872175f
#define ATAN_COEF6  0.1471039133652469f
#define ATAN_COEF7  0.6444640676891548f

	float abs_x = fabsf(x);
	float abs_y = fabsf(y);
	float res = MAX(abs_x, abs_y);

	if (res)
		res = MIN(abs_x, abs_y) / res;
	else
		res = 0.0f;

	res = -((((ATAN_COEF5 * res - ATAN_COEF4) * res -
		   ATAN_COEF3) * res - ATAN_COEF2) * res -
		   ATAN_COEF1) / ((ATAN_COEF7 * res + ATAN_COEF6) *
		   res + 1.0f);

	if (abs_y > abs_x)
		res = (0.5f * MATHS_PI_F) - res;
	if (x < 0.0f)
		res = MATHS_PI_F - res;
	if (y < 0.0f)
		res = -res;
	return res;
}

float maths_acos(float x)
{
	float xa = fabsf(x);
	float result = sqrtf(1.0f - xa) *
		(1.5707288f + xa * (-0.2121144f +
		 xa * (0.0742610f + (-0.0187293f * xa))));
	return (x < 0.0f) ? (MATHS_PI_F - result) : result;
}

#endif /* MATHS_NO_FAST_TRIG */

/* --- running standard deviation (Welford) --- */

void maths_stdev_clear(maths_stdev_t *dev)
{
	dev->m_n = 0;
}

void maths_stdev_push(maths_stdev_t *dev, float x)
{
	dev->m_n++;
	if (dev->m_n == 1) {
		dev->m_old_m = dev->m_new_m = x;
		dev->m_old_s = 0.0f;
	} else {
		dev->m_new_m = dev->m_old_m + (x - dev->m_old_m) / dev->m_n;
		dev->m_new_s = dev->m_old_s +
			       (x - dev->m_old_m) * (x - dev->m_new_m);
		dev->m_old_m = dev->m_new_m;
		dev->m_old_s = dev->m_new_s;
	}
}

float maths_stdev_variance(const maths_stdev_t *dev)
{
	return (dev->m_n > 1) ? (dev->m_new_s / (dev->m_n - 1)) : 0.0f;
}

float maths_stdev_stddev(const maths_stdev_t *dev)
{
	return sqrtf(maths_stdev_variance(dev));
}

/* --- constrain --- */

float maths_constrain_f(float val, float lo, float hi)
{
	if (val < lo)
		return lo;
	if (val > hi)
		return hi;
	return val;
}

int32_t maths_constrain_i(int32_t val, int32_t lo, int32_t hi)
{
	if (val < lo)
		return lo;
	if (val > hi)
		return hi;
	return val;
}

/* --- deadband --- */

float maths_deadband_f(float val, float deadband)
{
	if (fabsf(val) < deadband)
		return 0.0f;
	return (val > 0.0f) ? (val - deadband) : (val + deadband);
}

int32_t maths_deadband_i(int32_t val, int32_t deadband)
{
	if (ABS(val) < deadband)
		return 0;
	return (val > 0) ? (val - deadband) : (val + deadband);
}

/* --- scale range --- */

float maths_scale_f(float x,
		    float src_min, float src_max,
		    float dst_min, float dst_max)
{
	return (dst_max - dst_min) * (x - src_min) /
	       (src_max - src_min) + dst_min;
}

/* --- angle wrapping (degrees) --- */

float maths_wrap_180(float angle)
{
	while (angle >  180.0f) angle -= 360.0f;
	while (angle < -180.0f) angle += 360.0f;
	return angle;
}

float maths_wrap_360(float angle)
{
	while (angle >= 360.0f) angle -= 360.0f;
	while (angle <    0.0f) angle += 360.0f;
	return angle;
}

/* --- vector / rotation --- */

void maths_vec3_normalize(const Axis3f *src, Axis3f *dst)
{
	float len = sqrtf(src->x * src->x +
			  src->y * src->y +
			  src->z * src->z);
	if (len > 0.0f) {
		dst->x = src->x / len;
		dst->y = src->y / len;
		dst->z = src->z / len;
	}
}

void maths_rotation_matrix(const EulerAngle *euler, float m[3][3])
{
	float cx = maths_cos(euler->roll);
	float sx = maths_sin(euler->roll);
	float cy = maths_cos(euler->pitch);
	float sy = maths_sin(euler->pitch);
	float cz = maths_cos(euler->yaw);
	float sz = maths_sin(euler->yaw);

	float cz_cx = cz * cx;
	float sz_cx = sz * cx;
	float cz_sx = sx * cz;
	float sz_sx = sx * sz;

	m[0][AXIS_X] =  cz * cy;
	m[0][AXIS_Y] = -cy * sz;
	m[0][AXIS_Z] =  sy;

	m[1][AXIS_X] =  sz_cx + cz_sx * sy;
	m[1][AXIS_Y] =  cz_cx - sz_sx * sy;
	m[1][AXIS_Z] = -sx * cy;

	m[2][AXIS_X] =  sz_sx - cz_cx * sy;
	m[2][AXIS_Y] =  cz_sx + sz_cx * sy;
	m[2][AXIS_Z] =  cy * cx;
}

void maths_vec3_rotate(Axis3f *v, const EulerAngle *delta)
{
	Axis3f tmp = *v;
	float m[3][3];

	maths_rotation_matrix(delta, m);

	v->x = tmp.x * m[0][AXIS_X] + tmp.y * m[1][AXIS_X] +
	       tmp.z * m[2][AXIS_X];
	v->y = tmp.x * m[0][AXIS_Y] + tmp.y * m[1][AXIS_Y] +
	       tmp.z * m[2][AXIS_Y];
	v->z = tmp.x * m[0][AXIS_Z] + tmp.y * m[1][AXIS_Z] +
	       tmp.z * m[2][AXIS_Z];
}

/* --- median filter (sorting network) --- */

#define MF_SORT(a, b) do { \
	if ((a) > (b)) { float t_ = (a); (a) = (b); (b) = t_; } \
} while (0)

float maths_median_f3(float *v)
{
	float p[3];
	memcpy(p, v, sizeof(p));
	MF_SORT(p[0], p[1]);
	MF_SORT(p[1], p[2]);
	MF_SORT(p[0], p[1]);
	return p[1];
}

float maths_median_f5(float *v)
{
	float p[5];
	memcpy(p, v, sizeof(p));
	MF_SORT(p[0], p[1]);
	MF_SORT(p[3], p[4]);
	MF_SORT(p[0], p[3]);
	MF_SORT(p[1], p[4]);
	MF_SORT(p[1], p[2]);
	MF_SORT(p[2], p[3]);
	MF_SORT(p[1], p[2]);
	return p[2];
}

float maths_median_f7(float *v)
{
	float p[7];
	memcpy(p, v, sizeof(p));
	MF_SORT(p[0], p[5]);
	MF_SORT(p[0], p[3]);
	MF_SORT(p[1], p[6]);
	MF_SORT(p[2], p[4]);
	MF_SORT(p[0], p[1]);
	MF_SORT(p[3], p[5]);
	MF_SORT(p[2], p[6]);
	MF_SORT(p[2], p[3]);
	MF_SORT(p[3], p[6]);
	MF_SORT(p[4], p[5]);
	MF_SORT(p[1], p[4]);
	MF_SORT(p[1], p[3]);
	MF_SORT(p[3], p[4]);
	return p[3];
}

/* --- utility --- */

float maths_bell_curve(float x, float width)
{
	return powf(MATHS_E_F, -SQ(x) / (2.0f * SQ(width)));
}
