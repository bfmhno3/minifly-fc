// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package interface for four-channel motor PWM outputs.
 *
 * @details
 * This module exposes a stable board-level API for quadcopter motor control,
 * abstracting the underlying timer and channel mapping complexity.
 *
 * [Hardware Dependencies]
 * - TIM2 (CH1, CH3) and TIM4 (CH1, CH2) configured for PWM output.
 * - All timers assume the same PWM period configuration.
 * - No DMA or interrupt handling; direct HAL register access via compare updates.
 *
 * [Thread Safety]
 * This module is not thread-safe. If called from multiple tasks in a RTOS
 * environment, external synchronization (mutex or critical section) is required.
 */

#ifndef BSP_MOTORS_H
#define BSP_MOTORS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  BSP_MOTOR_1 = 0,
  BSP_MOTOR_2 = 1,
  BSP_MOTOR_3 = 2,
  BSP_MOTOR_4 = 3,
  BSP_MOTOR_COUNT = 4
};

/**
 * @brief Initialize the motor output module.
 *
 * Clears all motor compare values and marks the module as initialized.
 * Must be called once before any other motor API calls.
 *
 * @pre [TIM2_Init] and [TIM4_Init] must have already completed.
 * @note Idempotent: safe to call multiple times.
 */
void bsp_motors_init(void);

/**
 * @brief Start PWM generation on all motor channels.
 *
 * Enables the underlying hardware PWM generators. Motors are safely held at
 * zero thrust until an explicit bsp_motors_set_ratio() call.
 *
 * @pre bsp_motors_init() must have been called first.
 * @note Has no effect if already started or not initialized.
 */
void bsp_motors_start(void);

/**
 * @brief Stop all motor outputs and disable PWM generation.
 *
 * Safely de-energizes all motors by writing zero ratio to each channel,
 * then stops the underlying PWM generators.
 *
 * @pre bsp_motors_init() must have been called first.
 * @note Safe to call even if motors are not running.
 */
void bsp_motors_stop_all(void);

/**
 * @brief Set the output ratio for one motor channel.
 *
 * Maps a normalized 16-bit ratio [0, 65535] to the PWM duty cycle.
 * Value 0 produces 0% duty cycle (motor off), value 65535 produces
 * approximately 100% duty cycle (full throttle).
 *
 * @pre bsp_motors_init() must have been called first.
 * @param[in] id Motor identifier: [0, BSP_MOTOR_COUNT).
 * @param[in] ratio Throttle setting in the range [0, 65535].
 *
 * @note Has no effect if id is out of range or module is not initialized.
 * @note Ratio is internally scaled to the timer compare register using the
 *       configured PWM period; compare values are clamped to the period.
 */
void bsp_motors_set_ratio(uint8_t id, uint16_t ratio);

#ifdef __cplusplus
}
#endif

#endif
