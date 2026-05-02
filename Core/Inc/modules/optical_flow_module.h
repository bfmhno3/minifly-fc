#ifndef MODULES_OPTICAL_FLOW_MODULE_H
#define MODULES_OPTICAL_FLOW_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"

typedef struct {
    bool valid;              /* data is valid and usable */
    float pos_sum[2];        /* accumulated position, cm [x, y] */
    float vel_lpf[2];        /* filtered velocity, cm/s [x, y] */
    float laser_range;       /* laser rangefinder distance, cm */
    float laser_quality;     /* laser rangefinder quality [0.0, 1.0] */
} optical_flow_data_t;

void optical_flow_module_init(void);
void optical_flow_module_deinit(void);
void optical_flow_module_update(state_t *state, float dt);
bool optical_flow_module_is_valid(void);
void optical_flow_module_get_data(optical_flow_data_t *out);

#endif /* MODULES_OPTICAL_FLOW_MODULE_H */
