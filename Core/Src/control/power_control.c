// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  X-quad motor mixer implementation
 *
 * @details
 * Mixes roll/pitch/yaw PID outputs with thrust into four motor channels.
 * Motor layout (X-configuration, viewed from above):
 *   M1 (front-left)   M2 (front-right)
 *   M4 (rear-left)    M3 (rear-right)
 */

#include "control/power_control.h"

#include "bsp_motors.h"

static bool override_enabled;
static motor_pwm_t override_pwm;
static motor_pwm_t current_pwm;

static uint16_t clamp_u16(int32_t value)
{
  if (value > UINT16_MAX)
    return UINT16_MAX;
  if (value < 0)
    return 0;
  return (uint16_t)value;
}

void power_control_init(void)
{
  bsp_motors_init();
  /* NOTE: motors_start() is called before sensors are initialized.
   * This is safe because bsp_motors_init() zeros all PWM channels first,
   * but a future refactor should defer start until stabilizer_task begins. */
  bsp_motors_start();

  override_enabled = false;
  override_pwm = (motor_pwm_t){ 0, 0, 0, 0 };
  current_pwm = (motor_pwm_t){ 0, 0, 0, 0 };
}

bool power_control_test(void)
{
  return true;
}

void power_control_run(const control_t *ctl)
{
  motor_pwm_t pwm;

  if (override_enabled) {
    pwm = override_pwm;
  } else {
    /* X-quad mixing: each motor gets thrust +/- roll/pitch/yaw.
		 * Roll/pitch are halved so the sum of corrections stays bounded.
		 * Adjacent motors get opposite yaw signs for torque reaction. */
    int32_t r = (int32_t)(ctl->roll / 2);
    int32_t p = (int32_t)(ctl->pitch / 2);
    int32_t thrust = (int32_t)ctl->thrust;
    int32_t yaw = (int32_t)ctl->yaw;

    pwm.m1 = clamp_u16(thrust - r - p + yaw);
    pwm.m2 = clamp_u16(thrust - r + p - yaw);
    pwm.m3 = clamp_u16(thrust + r + p + yaw);
    pwm.m4 = clamp_u16(thrust + r - p - yaw);
  }

  current_pwm = pwm;

  bsp_motors_set_ratio(BSP_MOTOR_1, pwm.m1);
  bsp_motors_set_ratio(BSP_MOTOR_2, pwm.m2);
  bsp_motors_set_ratio(BSP_MOTOR_3, pwm.m3);
  bsp_motors_set_ratio(BSP_MOTOR_4, pwm.m4);
}

void power_control_get_pwm(motor_pwm_t *out)
{
  *out = current_pwm;
}

void power_control_set_override(bool enable, uint16_t m1, uint16_t m2,
                                uint16_t m3, uint16_t m4)
{
  override_enabled = enable;
  override_pwm.m1 = m1;
  override_pwm.m2 = m2;
  override_pwm.m3 = m3;
  override_pwm.m4 = m4;
}
