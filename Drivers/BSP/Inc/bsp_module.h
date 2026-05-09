#ifndef BSP_MODULE_H
#define BSP_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package interface for external module detect and power control.
 *
 * This module provides the board-level primitives needed by the future module
 * manager. It exposes raw ADC sampling for the module detect pin and unified
 * power control for the shared module power pin.
 */

typedef enum {
  BSP_MODULE_NONE = 0,
  BSP_MODULE_LED_RING = 1,
  BSP_MODULE_WIFI_CAMERA = 2,
  BSP_MODULE_OPTICAL_FLOW = 3,
  BSP_MODULE_RESERVED_1 = 4,
} bsp_module_id_t;

/**
 * @brief Initialize the external module BSP.
 *
 * This function applies a safe default state to the shared module power pin.
 * It does not perform module identification or module-specific bus switching.
 */
void bsp_module_init(void);

/**
 * @brief Read the averaged raw ADC value from the module detect pin.
 *
 * The returned value is the average of a small burst of software-triggered ADC
 * conversions on PB1 / ADC1_IN9.
 *
 * @return Averaged 12-bit raw ADC value in the range [0, 4095].
 *
 * @note This function returns 0 if the BSP has not been initialized or if ADC
 * sampling fails.
 */
uint16_t bsp_module_detect_read_raw(void);

/**
 * @brief Control the shared power pin for external modules.
 *
 * @param on true to enable module power, false to disable it.
 *
 * @note This function only controls the shared power pin. Module-specific bus
 * selection and identification state machines belong to higher layers.
 */
void bsp_module_power_set(bool on);

#ifdef __cplusplus
}
#endif

#endif
