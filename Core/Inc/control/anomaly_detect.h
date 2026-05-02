#ifndef CONTROL_ANOMALY_DETECT_H
#define CONTROL_ANOMALY_DETECT_H

#include <stdbool.h>
#include "control/flight_types.h"
#include "services/sensors_types.h"

void anomaly_detect_init(void);
bool anomaly_detect_check(const sensor_data_t *sensor,
			   const state_t *state,
			   const control_t *ctl);

#endif /* CONTROL_ANOMALY_DETECT_H */
