#ifndef CONTROL_POWER_CONTROL_H
#define CONTROL_POWER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
} motor_pwm_t;

void power_control_init(void);
bool power_control_test(void);
void power_control_run(const control_t *ctl);

void power_control_get_pwm(motor_pwm_t *out);
void power_control_set_override(bool enable,
				uint16_t m1, uint16_t m2,
				uint16_t m3, uint16_t m4);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_POWER_CONTROL_H */
