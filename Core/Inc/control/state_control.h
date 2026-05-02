#ifndef CONTROL_STATE_CONTROL_H
#define CONTROL_STATE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"
#include "services/sensors_types.h"

void state_control_init(void);
bool state_control_test(void);

void state_control_run(control_t *out, const sensor_data_t *sensor,
		       const state_t *state, const setpoint_t *sp,
		       uint32_t tick);

#endif /* CONTROL_STATE_CONTROL_H */
