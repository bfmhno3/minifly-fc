#ifndef CONTROL_POSITION_ESTIMATOR_H
#define CONTROL_POSITION_ESTIMATOR_H

#include "control/flight_types.h"
#include "services/sensors_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	bool valid;
	float pos_sum[2];
	float vel_lpf[2];
	float laser_range;
	float laser_quality;
} pos_estimator_ext_t;

void position_estimator_init(void);
void position_estimator_update(state_t *state, const sensor_data_t *sensor,
	const pos_estimator_ext_t *ext, float dt);
void estimator_reset_height(void);
void estimator_reset_all(void);
float position_estimator_get_fused_height(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_POSITION_ESTIMATOR_H */
