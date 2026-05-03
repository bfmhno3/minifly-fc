// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Independent watchdog (IWDG) driver implementation.
 *
 * @details
 * Delegates to CubeMX-generated MX_IWDG_Init() for peripheral setup and
 * HAL_IWDG_Refresh() for the kick operation.  Tracks initialization state
 * so bsp_watchdog_kick() can auto-detect a running IWDG.
 */
#include "bsp_watchdog.h"

#include "iwdg.h"

#include <stdbool.h>

static bool watchdog_initialized = false;

void bsp_watchdog_init(uint32_t timeout_ms)
{
    (void)timeout_ms;

    MX_IWDG_Init();
    watchdog_initialized = true;
}

void bsp_watchdog_kick(void)
{
    /* Auto-detect: if IWDG is already running (e.g. CubeMX started it), mark initialized */
    if (!watchdog_initialized && (hiwdg.Instance == IWDG))
    {
        watchdog_initialized = true;
    }

    if (!watchdog_initialized)
    {
        return;
    }

    (void)HAL_IWDG_Refresh(&hiwdg);
}
