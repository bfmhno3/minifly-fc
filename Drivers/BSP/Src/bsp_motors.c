// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package implementation for four-channel motor PWM outputs.
 *
 * @details
 * Internal state machine and timer/channel mapping are fully hidden from the
 * caller. This layer handles ratio normalization, timer lookups, and safe
 * initialization/startup sequencing.
 *
 * [Ratio Scaling]
 * Input ratio [0, 65535] is scaled to the timer compare value [0, period].
 * This linear mapping ensures feedback-friendly throttle curves across varying
 * PWM periods. Clamping prevents register overflow.
 */

#include "bsp_motors.h"

#include <stdbool.h>
#include <stddef.h>

#include "tim.h"

typedef struct bsp_motor_output {
  TIM_HandleTypeDef *timer;
  uint32_t channel;
} bsp_motor_output_t;

// Static mapping of motor indices to STM32 timer resources.
// Motor 1,2 -> TIM4; Motor 3,4 -> TIM2 for balanced load distribution.
static const bsp_motor_output_t motor_outputs[BSP_MOTOR_COUNT] = {
  { &htim4, TIM_CHANNEL_2 },
  { &htim4, TIM_CHANNEL_1 },
  { &htim2, TIM_CHANNEL_3 },
  { &htim2, TIM_CHANNEL_1 },
};

static bool motors_initialized = false;
static bool motors_started = false;

/**
 * @brief Fetch the PWM period (auto-reload register value).
 *
 * Reads the timer period from TIM2. All motor timers are assumed to share
 * the same PWM frequency for safe inter-timer portability.
 *
 * @return Auto-reload register value used as the PWM period in clock ticks.
 */
static uint32_t bsp_motors_get_period(void)
{
  return __HAL_TIM_GET_AUTORELOAD(&htim2);
}

/**
 * @brief Normalize a 16-bit ratio to the PWM timer compare register.
 *
 * The input ratio is treated as a fixed-point fraction of full scale (65535).
 * We multiply by (period + 1) to map [0, 65535] onto [0, period] inclusive,
 * then clamp to prevent register overflow during edge cases.
 *
 * @param[in] ratio Normalized throttle value in the range [0, 65535].
 * @return Timer compare register value clamped to [0, period].
 */
static uint32_t bsp_motors_scale_ratio(uint16_t ratio)
{
  const uint32_t period = bsp_motors_get_period();
  const uint32_t scaled_ratio = (uint32_t)ratio * (period + 1u);
  const uint32_t compare_value = scaled_ratio / 65536u;

  // Clamp to prevent compare register overflow if rounding pushes past period.
  if (compare_value > period) {
    return period;
  }

  return compare_value;
}

/**
 * @brief Check if a motor identifier is within valid range.
 *
 * @param[in] id Motor index to validate.
 * @return true if id is in [0, BSP_MOTOR_COUNT), false otherwise.
 */
static bool bsp_motors_is_valid_id(uint8_t id)
{
  return id < BSP_MOTOR_COUNT;
}

/**
 * @brief Write a new throttle ratio to a single motor channel.
 *
 * Looks up the timer/channel pair, scales the ratio, and updates the
 * hardware compare register.
 *
 * @param[in] output Pointer to the motor's timer/channel configuration.
 * @param[in] ratio Normalized throttle value [0, 65535].
 */
static void bsp_motors_write_channel(const bsp_motor_output_t *output,
                                     uint16_t ratio)
{
  __HAL_TIM_SET_COMPARE(output->timer, output->channel,
                        bsp_motors_scale_ratio(ratio));
}

/**
 * @brief Broadcast a throttle ratio to all four motors.
 *
 * Used during initialization (set all to zero) and shutdown (de-energize all).
 *
 * @param[in] ratio Throttle value to write to all channels [0, 65535].
 */
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
