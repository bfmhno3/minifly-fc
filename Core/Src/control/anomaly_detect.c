#include "control/anomaly_detect.h"
#include <math.h>

#define ANOMALY_FF_THRESHOLD         0.05f
#define ANOMALY_FF_COUNT             50
#define ANOMALY_TUMBLE_THRESHOLD_DEG 60.0f
#define ANOMALY_TUMBLE_COUNT         100

static uint16_t ff_cnt;
static uint16_t tumble_cnt;
static uint16_t flip_cooldown_cnt;

static bool detect_free_fall(const Axis3f *acc)
{
	float acc_mag = acc->x * acc->x + acc->y * acc->y + acc->z * acc->z;

	if (acc_mag < ANOMALY_FF_THRESHOLD) {
		if (++ff_cnt >= ANOMALY_FF_COUNT)
			return true;
	} else {
		ff_cnt = 0;
	}

	return false;
}

static bool detect_tumbled(const attitude_t *att)
{
	float abs_roll  = fabsf(att->roll);
	float abs_pitch = fabsf(att->pitch);
	float max_angle = (abs_roll >= abs_pitch) ? abs_roll : abs_pitch;

	if (max_angle > ANOMALY_TUMBLE_THRESHOLD_DEG) {
		if (++tumble_cnt >= ANOMALY_TUMBLE_COUNT)
			return true;
	} else {
		tumble_cnt = 0;
	}

	return false;
}

void anomaly_detect_init(void)
{
	ff_cnt = 0;
	tumble_cnt = 0;
	flip_cooldown_cnt = 0;
}

bool anomaly_detect_check(const sensor_data_t *sensor,
			   const state_t *state,
			   const control_t *ctl)
{
	if (ctl->flip_dir != FLIP_DIR_CENTER) {
		flip_cooldown_cnt = 1000;
		return false;
	}

	if (flip_cooldown_cnt > 0) {
		flip_cooldown_cnt--;
		return false;
	}

	if (detect_free_fall(&sensor->acc))
		return true;

	if (detect_tumbled(&state->attitude))
		return true;

	return false;
}
