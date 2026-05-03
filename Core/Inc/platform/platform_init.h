// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Top-level platform initialization orchestrator.
 *
 * @details
 * Provides the single entry point for boot-time initialization of all
 * platform subsystems in the correct dependency order.
 */

#ifndef PLATFORM_INIT_H
#define PLATFORM_INIT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize all platform subsystems in dependency order.
 *
 * Call once at boot before any service or control module.
 *
 * @pre    HAL and CubeMX peripheral init must have completed.
 */
void platform_init(void);

/**
 * @brief  Run a post-init sanity check.
 *
 * @retval true   Platform is ready (sensors initialized successfully).
 * @retval false  Sensor initialization failed.
 */
bool platform_self_test(void);

#ifdef __cplusplus
}
#endif

#endif
