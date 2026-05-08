// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  WS2812 LED ring expansion module.
 *
 * @details
 * Drives a 12-LED WS2812 ring via bsp_ws2812.  A FreeRTOS software timer
 * (50 ms period) updates the LED buffer at 20 fps.  Multiple visual effects
 * are selectable; each effect function writes directly into a shared RGB
 * buffer that is pushed to the LEDs on every timer tick.
 *
 * The module is designed to be managed by module_manager -- it is
 * initialized when an LED ring is detected on the module connector and
 * de-initialized when it is removed.
 */

#ifndef MODULES_LEDRING_MODULE_H
#define MODULES_LEDRING_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Available LED ring visual effects.
 */
typedef enum {
  LEDRING_EFFECT_OFF = 0,     /**< All LEDs off. */
  LEDRING_EFFECT_COLOR_TEST,  /**< Cycles through a 9-color test palette. */
  LEDRING_EFFECT_ATTITUDE,    /**< Pitch/roll mapped to LED brightness. */
  LEDRING_EFFECT_GYRO,        /**< Angular rate mapped to RGB channels. */
  LEDRING_EFFECT_BLINK,       /**< Periodic blue blink. */
  LEDRING_EFFECT_FLASHLIGHT,  /**< All LEDs full white. */
  LEDRING_EFFECT_BREATHING,   /**< Breathing effect cycling R/G/B. */
  LEDRING_EFFECT_RED_SPIN,    /**< Red gradient spinning clockwise. */
  LEDRING_EFFECT_COLOR_SPIN,  /**< Multi-color gradient spinning clockwise. */
  LEDRING_EFFECT_DOUBLE_SPIN, /**< Two gradients counter-rotating, merged. */
  LEDRING_EFFECT_COUNT,       /**< Number of effects (sentinel). */
} ledring_effect_t;

/**
 * @brief  Initialize the LED ring module.
 *
 * Sets up WS2812 hardware, enables the LED power rail, and starts the
 * FreeRTOS software timer.  Safe to call multiple times (re-init path
 * restarts the timer without re-creating it).
 */
void ledring_module_init(void);

/**
 * @brief  De-initialize the LED ring module.
 *
 * Stops the timer, sends an all-black frame, disables the headlight,
 * and cuts power to the LED ring.
 */
void ledring_module_deinit(void);

/**
 * @brief  Request a visual effect change.
 *
 * The change takes effect on the next timer tick.  Invalid values are
 * silently ignored.
 *
 * @param[in] effect  Desired effect from ledring_effect_t.
 */
void ledring_module_set_effect(ledring_effect_t effect);

/**
 * @brief  Get the currently active effect.
 *
 * @return The effect being rendered (may differ from the last requested
 *         effect if the timer has not yet fired).
 */
ledring_effect_t ledring_module_get_effect(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULES_LEDRING_MODULE_H */
