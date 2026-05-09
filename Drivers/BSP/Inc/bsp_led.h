// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board-level API for controlling on-board status LEDs.
 *
 * This header exposes the logical LED interface. GPIO mapping and electrical
 * polarity are intentionally hidden in the BSP implementation.
 */

#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  BSP_LED_BLUE_L = 0,
  BSP_LED_GREEN_L = 1,
  BSP_LED_RED_L = 2,
  BSP_LED_GREEN_R = 3,
  BSP_LED_RED_R = 4,
  BSP_LED_COUNT = 5
};

/**
 * @brief Initialize the LED module and force all LEDs to logical off.
 *
 * @pre GPIO for all LED pins must already be initialized by board startup code.
 */
void bsp_led_init(void);

/**
 * @brief Set one LED to a logical state.
 *
 * @param[in] id LED identifier in range [0, BSP_LED_COUNT).
 * @param[in] on Logical target state, true for on and false for off.
 *
 * @note Call is ignored if the module is not initialized or if `id` is out of range.
 */
void bsp_led_set(uint8_t id, bool on);

/**
 * @brief Toggle one LED logical state.
 *
 * @param[in] id LED identifier in range [0, BSP_LED_COUNT).
 *
 * @note Call is ignored if the module is not initialized or if `id` is out of range.
 */
void bsp_led_toggle(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif
