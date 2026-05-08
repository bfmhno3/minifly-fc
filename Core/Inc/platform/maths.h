// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Platform math library: fast trig, statistics, vector operations.
 *
 * @details
 * Provides order-9 Chebyshev polynomial trig approximations (disable with
 * MATHS_NO_FAST_TRIG to fall back to standard libm), Welford online
 * statistics, rotation matrices, and median filters using optimal sorting
 * networks. No hardware dependencies -- pure math.
 */

#ifndef PLATFORM_MATHS_H
#define PLATFORM_MATHS_H

#include <stdint.h>
#include <math.h>

#include "platform/axis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- constants --- */

#define MATHS_PI_F 3.14159265358979323846f
#define MATHS_E_F 2.71828182845904523536f
#define MATHS_RAD (MATHS_PI_F / 180.0f) /* degrees-to-radians multiplier */

/* --- basic macros --- */

#ifndef SQ
#define SQ(x) ((x) * (x))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(x) (((x) < 0) ? (-(x)) : (x))
#endif

/* --- angle conversion macros --- */

#define DEG_TO_RAD(a) ((a) * MATHS_RAD)
#define RAD_TO_DEG(a) ((a) / MATHS_RAD)

/* --- fast trig (order-9 Chebyshev approximation) --- */

#ifndef MATHS_NO_FAST_TRIG
float maths_sin(float x);
float maths_cos(float x);
float maths_atan2(float y, float x);
float maths_acos(float x);
#define maths_tan(x) (maths_sin(x) / maths_cos(x))
#else
#define maths_sin(x) sinf(x)
#define maths_cos(x) cosf(x)
#define maths_atan2(y, x) atan2f(y, x)
#define maths_acos(x) acosf(x)
#define maths_tan(x) tanf(x)
#endif

/* --- running standard deviation (Welford) --- */

/**
 * @brief  Running standard deviation state (Welford's online algorithm).
 *
 * Numerically stable incremental variance computation -- avoids
 * catastrophic cancellation from large intermediate sums.
 */
typedef struct maths_stdev {
  float m_old_m;
  float m_new_m;
  float m_old_s;
  float m_new_s;
  int m_n;
} maths_stdev_t;

/** @brief  Reset the accumulator to empty state. */
void maths_stdev_clear(maths_stdev_t *dev);

/** @brief  Add a sample using Welford's recurrence. */
void maths_stdev_push(maths_stdev_t *dev, float x);

/** @brief  Return the sample variance (n-1 denominator). Returns 0 if n < 2. */
float maths_stdev_variance(const maths_stdev_t *dev);

/** @brief  Return the sample standard deviation. */
float maths_stdev_stddev(const maths_stdev_t *dev);

/* --- constrain --- */

/** @brief  Clamp float to [lo, hi]. */
float maths_constrain_f(float val, float lo, float hi);
/** @brief  Clamp int32 to [lo, hi]. */
int32_t maths_constrain_i(int32_t val, int32_t lo, int32_t hi);

/* --- deadband --- */

/** @brief  Deadband: returns 0 within +/-deadband, offset output outside. */
float maths_deadband_f(float val, float deadband);
/** @brief  Integer deadband. */
int32_t maths_deadband_i(int32_t val, int32_t deadband);

/* --- scale range --- */

/** @brief  Linear interpolation/extrapolation from [src_min, src_max] to [dst_min, dst_max]. */
float maths_scale_f(float x, float src_min, float src_max, float dst_min,
                    float dst_max);

/* --- angle wrapping (degrees) --- */

/** @brief  Wrap angle in degrees to [-180, 180]. */
float maths_wrap_180(float angle);
/** @brief  Wrap angle in degrees to [0, 360). */
float maths_wrap_360(float angle);

/* --- vector / rotation --- */

/**
 * @brief  Euler angles in degrees (roll, pitch, yaw).
 *
 * Used as input to maths_rotation_matrix() which applies ZYX intrinsic
 * convention: yaw first, then pitch, then roll.
 */
typedef struct euler_angle {
  float roll;
  float pitch;
  float yaw;
} euler_angle_t;

/** @brief  Normalize a 3D vector. No-op if length is zero. */
void maths_vec3_normalize(const axis3f_t *src, axis3f_t *dst);

/** @brief  Build a 3x3 rotation matrix from Euler angles (ZYX intrinsic). */
void maths_rotation_matrix(const euler_angle_t *euler, float matrix[3][3]);

/** @brief  Rotate vector v in-place by Euler angle delta. */
void maths_vec3_rotate(axis3f_t *v, const euler_angle_t *delta);

/* --- median filter --- */

/**
 * @brief  Median of 3 values using optimal sorting network (3 comparisons).
 * @note   Overwrites the input array; pass a copy if the original is needed.
 */
float maths_median_f3(float *v);
/** @brief  Median of 5 values (9 comparisons). */
float maths_median_f5(float *v);
/** @brief  Median of 7 values (16 comparisons). */
float maths_median_f7(float *v);

/* --- utility --- */

/** @brief  Gaussian-like bell curve: exp(-x^2 / (2 * width^2)). */
float maths_bell_curve(float x, float width);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_MATHS_H */
