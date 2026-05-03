// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Independent watchdog (IWDG) abstraction for the Minifly board.
 *
 * @details
 * Wraps CubeMX's MX_IWDG_Init() and HAL_IWDG_Refresh().  The timeout
 * parameter is accepted for API compatibility but the actual period is
 * determined by CubeMX IWDG configuration.
 */

#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the independent watchdog.
 *
 * @param[in] timeout_ms  Requested timeout in milliseconds (currently unused;
 *                        actual period is set by CubeMX IWDG config).
 *
 * @note Must be called before bsp_watchdog_kick().
 */
void bsp_watchdog_init(uint32_t timeout_ms);

/**
 * @brief Refresh (kick) the watchdog to prevent reset.
 *
 * @note If the watchdog was not explicitly initialized but the IWDG peripheral
 *       is running (e.g. started by CubeMX), this function marks the module as
 *       initialized and proceeds with the refresh.
 */
void bsp_watchdog_kick(void);

#ifdef __cplusplus
}
#endif

#endif
