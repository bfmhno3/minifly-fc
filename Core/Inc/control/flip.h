#ifndef CONTROL_FLIP_H
#define CONTROL_FLIP_H

#include <stdbool.h>
#include "control/flight_types.h"

void flip_init(void);
void flip_set_dir(flip_dir_e dir);
void flip_check(setpoint_t *sp, control_t *ctl, const state_t *state);

#endif /* CONTROL_FLIP_H */
