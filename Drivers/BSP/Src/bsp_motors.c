// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package implementation for the four motor PWM outputs.
 *
 * The implementation keeps the timer/channel mapping, initialization state, and
 * ratio-to-compare scaling local to the BSP layer.
 */
#include "bsp_motors.h"

#include "tim.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct {
  TIM_HandleTypeDef *timer;
  uint32_t channel;
} bsp_motor_output_t;

static const bsp_motor_output_t motor_outputs[BSP_MOTOR_COUNT] = {
  { &htim4, TIM_CHANNEL_2 },
  { &htim4, TIM_CHANNEL_1 },
  { &htim2, TIM_CHANNEL_3 },
  { &htim2, TIM_CHANNEL_1 },
};

static bool motors_initialized = false;
static bool motors_started = false;

/**
 * @brief Get the PWM period used to scale motor output ratios.
 *
 * @return Timer auto-reload value used as the PWM period.
 *
 * @note This implementation reads the period from TIM2 and assumes the motor
 * timers share the same PWM period configuration.
 */
static uint32_t bsp_motors_get_period(void)
{
  return __HAL_TIM_GET_AUTORELOAD(&htim2);
}

/**
 * @brief Scale a 16-bit motor ratio to a timer compare value.
 *
 * @param ratio Unsigned 16-bit output ratio in the range [0, 65535].
 * @return Timer compare value clamped to the configured PWM period.
 */
static uint32_t bsp_motors_scale_ratio(uint16_t ratio)
{
  const uint32_t period = bsp_motors_get_period();
  const uint32_t scaled_ratio = (uint32_t)ratio * (period + 1u);
  const uint32_t compare_value = scaled_ratio / 65536u;

  if (compare_value > period) {
    return period;
  }

  return compare_value;
}

static bool bsp_motors_is_valid_id(uint8_t id)
{
  return id < BSP_MOTOR_COUNT;
}

static void bsp_motors_write_channel(const bsp_motor_output_t *output,
                                     uint16_t ratio)
{
  __HAL_TIM_SET_COMPARE(output->timer, output->channel,
                        bsp_motors_scale_ratio(ratio));
}

static void bsp_motors_write_all(uint16_t ratio)
{
  uint8_t index = 0u;

  for (; index < BSP_MOTOR_COUNT; ++index) {
    bsp_motors_write_channel(&motor_outputs[index], ratio);
  }
}

void bsp_motors_init(void)
{
  bsp_motors_write_all(0u);
  motors_initialized = true;
}

void bsp_motors_start(void)
{
  uint8_t index = 0u;

  if (!motors_initialized || motors_started) {
    return;
  }

  for (; index < BSP_MOTOR_COUNT; ++index) {
    (void)HAL_TIM_PWM_Start(motor_outputs[index].timer,
                            motor_outputs[index].channel);
  }

  motors_started = true;
}

void bsp_motors_stop_all(void)
{
  uint8_t index = 0u;

  if (!motors_initialized) {
    return;
  }

  bsp_motors_write_all(0u);

  for (; index < BSP_MOTOR_COUNT; ++index) {
    (void)HAL_TIM_PWM_Stop(motor_outputs[index].timer,
                           motor_outputs[index].channel);
  }

  motors_started = false;
}

void bsp_motors_set_ratio(uint8_t id, uint16_t ratio)
{
  if (!motors_initialized || !bsp_motors_is_valid_id(id)) {
    return;
  }

  bsp_motors_write_channel(&motor_outputs[id], ratio);
}
