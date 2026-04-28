#ifndef SERVICES_SENSORS_H
#define SERVICES_SENSORS_H

#include "services/sensors_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void sensors_init(void);
bool sensors_test(void);
bool sensors_are_calibrated(void);
void sensors_task(void *arg);

void sensors_acquire(sensor_data_t *out, uint32_t tick);
bool sensors_read_gyro(Axis3f *gyro);
bool sensors_read_acc(Axis3f *acc);
bool sensors_read_baro(baro_t *baro);

void sensors_get_raw_data(Axis3i16 *acc, Axis3i16 *gyro, Axis3i16 *mag);
bool sensors_is_mpu_present(void);
bool sensors_is_baro_present(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_SENSORS_H */
