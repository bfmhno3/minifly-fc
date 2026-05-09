#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package interface for on-board status LEDs.
 *
 * This module exposes a board-level LED control API while keeping the GPIO
 * details and electrical polarity local to the BSP layer.
 */

enum {
  BSP_LED_BLUE_L = 0,
  BSP_LED_GREEN_L = 1,
  BSP_LED_RED_L = 2,
  BSP_LED_GREEN_R = 3,
  BSP_LED_RED_R = 4,
  BSP_LED_COUNT = 5
};

/**
 * @brief Initialize the LED module.
 *
 * This function drives every LED to the logical off state.
 */
void bsp_led_init(void);

/**
 * @brief Set the logical state of one LED.
 *
 * @param id LED identifier in the range [0, BSP_LED_COUNT).
 * @param on Logical LED state where true means on and false means off.
 *
 * @note This function has no effect if the module is not initialized or if the
 * LED identifier is invalid.
 */
void bsp_led_set(uint8_t id, bool on);

/**
 * @brief Toggle the logical state of one LED.
 *
 * @param id LED identifier in the range [0, BSP_LED_COUNT).
 *
 * @note This function has no effect if the module is not initialized or if the
 * LED identifier is invalid.
 */
void bsp_led_toggle(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif
