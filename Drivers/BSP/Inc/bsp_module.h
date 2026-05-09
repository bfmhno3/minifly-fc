// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package interface for external module detect and power control.
 *
 * @details
 * This module provides the board-level primitives for the external module subsystem.
 * It exposes raw ADC sampling for module identification and unified power control
 * for the shared module power pin.
 *
 * [Hardware Dependencies]
 * - Module detect pin: PB1 / ADC1_IN9 (analog input for module ID discrimination)
 * - Module power pin: GPIO output for enabling/disabling external module rail
 * - ADC1 configured for 12-bit conversion with software-triggered polling
 *
 * [Design Scope]
 * This BSP layer provides only raw I/O primitives. It does NOT implement:
 * - Module identification state machines (threshold-based classification)
 * - Module-specific bus switching (I2C/SPI multiplexing by module type)
 * These responsibilities belong to higher-level module manager layer.
 */

#ifndef BSP_MODULE_H
#define BSP_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief External module type identification via ADC thresholds.
 *
 * Each module variant presents a distinct ADC value on PB1, allowing firmware
 * to autonomously detect which hardware is attached without firmware reprogramming.
 */
typedef enum bsp_module_id {
  BSP_MODULE_NONE = 0,         // No module detected or slot unpopulated
  BSP_MODULE_LED_RING = 1,     // LED ring module detected
  BSP_MODULE_WIFI_CAMERA = 2,  // WiFi camera module detected
  BSP_MODULE_OPTICAL_FLOW = 3, // Optical flow sensor module detected
  BSP_MODULE_RESERVED_1 = 4,   // Reserved for future module variants
} bsp_module_id_t;

/**
 * @brief Initialize the external module BSP.
 *
 * @details
 * Applies a safe default state (power OFF) to the shared module power pin and
 * marks the BSP as ready. Subsequent API calls verify initialization state.
 *
 * @pre Must be called during system initialization, after STM32 HAL initialization
 * and before any other bsp_module_* calls.
 *
 * @note This function does not perform module detection or configure module-specific
 * hardware (bus selection, interrupt routing, etc.). It only ensures the shared
 * power pin is in a safe state.
 */
void bsp_module_init(void);

/**
 * @brief Read the averaged raw ADC value from the module detect pin.
 *
 * @details
 * Performs a small burst of software-triggered ADC conversions on PB1/ADC1_IN9,
 * accumulates the samples, and returns the mean value. Averaging reduces noise
 * from switching transients on the shared power rail.
 *
 * Typical ADC readings by module type:
 * - No module (open circuit): ~0 mV -> ADC ~0
 * - LED ring (2k2 divider): ~1000 mV -> ADC ~2048
 * - WiFi camera (open or pull-up weakly driven): ~5000 mV -> ADC ~4095
 * - Optical flow (1k4 divider): ~1380 mV -> ADC ~2815
 * - Reserved module (640 ohm divider): ~640 mV -> ADC ~1280
 *
 * @return Averaged 12-bit raw ADC value in the range [0, 4095].
 *
 * @retval 0 if BSP has not been initialized or if ADC sampling fails.
 *
 * @pre bsp_module_init() must have been called successfully.
 *
 * @note The caller (typically the module manager) is responsible for applying
 * ADC-to-module-ID classification logic with appropriate hysteresis.
 */
uint16_t bsp_module_detect_read_raw(void);

/**
 * @brief Control the shared power pin for external modules.
 *
 * @param[in] on true to enable module power rail, false to disable it.
 *
 * @pre bsp_module_init() must have been called successfully.
 *
 * @note
 * - This function owns only the shared power pin GPIO control.
 * - Module-specific bus selection, interrupt configuration, and identification
 *   logic are NOT handled by this BSP layer.
 * - Power state transitions should be coordinated with higher-level module
 *   manager state machine to avoid bus conflicts.
 */
void bsp_module_power_set(bool on);

#ifdef __cplusplus
}
#endif

#endif
