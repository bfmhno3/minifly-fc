#ifndef CONTROL_STABILIZER_H
#define CONTROL_STABILIZER_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"
#include "platform/axis.h"
#include "services/sensors_types.h"

void stabilizer_init(void);
bool stabilizer_test(void);
void stabilizer_task(void *arg);

void stabilizer_get_attitude(attitude_t *out);
float stabilizer_get_baro(void);
void stabilizer_get_sensor_data(sensor_data_t *out);
void stabilizer_get_state(Axis3f *acc, Axis3f *vel, Axis3f *pos);

void stabilizer_set_fast_adjust(uint16_t vel_ticks, uint16_t abs_ticks,
				 float height);

#endif /* CONTROL_STABILIZER_H */
