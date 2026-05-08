// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Motor mixing and PWM output control
 *
 * @details
 * Converts roll/pitch/yaw effort and thrust into four motor PWM values
 * using a standard X-quad mixing scheme.  Supports a manual override mode
 * for bench testing.
 */

#ifndef CONTROL_POWER_CONTROL_H
#define CONTROL_POWER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "control/flight_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief  Per-motor PWM values (raw timer compare units) */
typedef struct {
  uint16_t m1; /**< Front-left motor */
  uint16_t m2; /**< Front-right motor */
  uint16_t m3; /**< Rear-right motor */
  uint16_t m4; /**< Rear-left motor */
} motor_pwm_t;

void power_control_init(void);
bool power_control_test(void);

/**
 * @brief  Mix control effort into motor PWM and drive the motors
 * @param[in] ctl  Roll/pitch/yaw effort + thrust from the PID controller
 */
void power_control_run(const control_t *ctl);

/** @brief  Read back the last written PWM values */
void power_control_get_pwm(motor_pwm_t *out);

/**
 * @brief  Override motor outputs for bench testing
 *
 * When enabled, power_control_run() uses the provided PWM values
 * instead of the PID-computed mix.
 *
 * @param[in] enable  true to activate override, false to return to normal
 * @param[in] m1-m4   Direct PWM values per motor
 */
void power_control_set_override(bool enable, uint16_t m1, uint16_t m2,
                                uint16_t m3, uint16_t m4);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_POWER_CONTROL_H */
