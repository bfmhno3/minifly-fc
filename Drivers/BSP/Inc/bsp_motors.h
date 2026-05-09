// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package interface for the four motor PWM outputs.
 *
 * This module exposes a stable board-level API for motor output control while
 * keeping the underlying timer and channel mapping internal to the BSP layer.
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
 * This function clears all motor compare values before the outputs are started.
 */
void bsp_motors_init(void);

/**
 * @brief Start PWM generation on all motor channels.
 *
 * @note This function has no effect if the module is not initialized or if the
 * motor outputs have already been started.
 */
void bsp_motors_start(void);

/**
 * @brief Stop all motor outputs and disable PWM generation.
 *
 * @note This function first writes a zero ratio to every motor channel before
 * stopping the underlying PWM outputs.
 */
void bsp_motors_stop_all(void);

/**
 * @brief Set the output ratio for one motor channel.
 *
 * @param id Motor identifier in the range [0, BSP_MOTOR_COUNT).
 * @param ratio Unsigned 16-bit output ratio in the range [0, 65535].
 *
 * @note The ratio is scaled to the timer compare register using the configured
 * PWM period. The function has no effect if the module is not initialized or if
 * the motor identifier is invalid.
 */
void bsp_motors_set_ratio(uint8_t id, uint16_t ratio);

#ifdef __cplusplus
}
#endif

#endif
