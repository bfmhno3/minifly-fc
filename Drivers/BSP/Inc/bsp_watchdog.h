// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Independent watchdog (IWDG) abstraction for the Minifly board.
 *
 * @details
 * This module wraps CubeMX-generated IWDG primitives and exposes a small BSP API.
 *
 * Hardware dependency:
 * - Requires CubeMX IWDG setup in `iwdg.c` (`MX_IWDG_Init`) and global handle `hiwdg`.
 *
 * Design notes:
 * - `timeout_ms` is accepted for API compatibility with other platforms, but the actual
 *   watchdog period is defined by CubeMX prescaler/reload configuration.
 * - APIs are non-blocking and keep internal state to avoid refreshing before startup.
 */

#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the independent watchdog peripheral.
 *
 * @param[in] timeout_ms  Requested timeout in milliseconds. Currently unused by this
 *                        backend; real timeout is fixed by CubeMX IWDG configuration.
 *
 * @pre `MX_IWDG_Init()` must be available from CubeMX-generated code.
 * @attention Once started, IWDG cannot be stopped on STM32F4 and must be refreshed
 *            periodically to avoid a system reset.
 */
void bsp_watchdog_init(uint32_t timeout_ms);

/**
 * @brief Refresh (kick) the watchdog counter.
 *
 * @details
 * If the module was not initialized via `bsp_watchdog_init()` but IWDG is already
 * running (for example, started by system init code), this function auto-detects
 * that state and enables refresh.
 *
 * @pre IWDG must be started either by `bsp_watchdog_init()` or equivalent startup code.
 */
void bsp_watchdog_kick(void);

#ifdef __cplusplus
}
#endif

#endif
