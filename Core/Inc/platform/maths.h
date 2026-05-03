#ifndef PLATFORM_MATHS_H
#define PLATFORM_MATHS_H

#include <stdint.h>
#include <math.h>

#include "platform/axis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- constants --- */

#define MATHS_PI_F   3.14159265358979323846f
#define MATHS_E_F    2.71828182845904523536f
#define MATHS_RAD    (MATHS_PI_F / 180.0f)

/* --- basic macros --- */

#ifndef SQ
#define SQ(x)        ((x) * (x))
#endif

#ifndef MIN
#define MIN(a, b)    (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)    (((a) > (b)) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(x)       (((x) < 0) ? (-(x)) : (x))
#endif

/* --- angle conversion macros --- */

#define DEG_TO_RAD(a)   ((a) * MATHS_RAD)
#define RAD_TO_DEG(a)   ((a) / MATHS_RAD)

/* --- fast trig (order-9 Chebyshev approximation) --- */

#ifndef MATHS_NO_FAST_TRIG
float maths_sin(float x);
float maths_cos(float x);
float maths_atan2(float y, float x);
float maths_acos(float x);
#define maths_tan(x) (maths_sin(x) / maths_cos(x))
#else
#define maths_sin(x)       sinf(x)
#define maths_cos(x)       cosf(x)
#define maths_atan2(y, x)  atan2f(y, x)
#define maths_acos(x)      acosf(x)
#define maths_tan(x)       tanf(x)
#endif

/* --- running standard deviation (Welford) --- */

typedef struct {
	float m_old_m;
	float m_new_m;
	float m_old_s;
	float m_new_s;
	int m_n;
} maths_stdev_t;

void  maths_stdev_clear(maths_stdev_t *dev);
void  maths_stdev_push(maths_stdev_t *dev, float x);
float maths_stdev_variance(const maths_stdev_t *dev);
float maths_stdev_stddev(const maths_stdev_t *dev);

/* --- constrain --- */

float   maths_constrain_f(float val, float lo, float hi);
int32_t maths_constrain_i(int32_t val, int32_t lo, int32_t hi);

/* --- deadband --- */

float   maths_deadband_f(float val, float deadband);
int32_t maths_deadband_i(int32_t val, int32_t deadband);

/* --- scale range --- */

float maths_scale_f(float x,
		    float src_min, float src_max,
		    float dst_min, float dst_max);

/* --- angle wrapping (degrees) --- */

float maths_wrap_180(float angle);
float maths_wrap_360(float angle);

/* --- vector / rotation --- */

typedef struct {
	float roll;
	float pitch;
	float yaw;
} EulerAngle;

void maths_vec3_normalize(const Axis3f *src, Axis3f *dst);
void maths_rotation_matrix(const EulerAngle *euler, float matrix[3][3]);
void maths_vec3_rotate(Axis3f *v, const EulerAngle *delta);

/* --- median filter --- */

float maths_median_f3(float *v);
float maths_median_f5(float *v);
float maths_median_f7(float *v);

/* --- utility --- */

float maths_bell_curve(float x, float width);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_MATHS_H */
