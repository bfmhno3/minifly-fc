// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Independent watchdog (IWDG) BSP implementation.
 *
 * @details
 * Delegates peripheral setup to CubeMX-generated `MX_IWDG_Init()` and refresh
 * to `HAL_IWDG_Refresh()`. A local initialization flag is used to prevent
 * unintended refresh attempts before IWDG startup is confirmed.
 */

#include "bsp_watchdog.h"

#include <stdbool.h>

#include "iwdg.h"

static bool watchdog_initialized = false;

void bsp_watchdog_init(uint32_t timeout_ms)
{
  (void)timeout_ms;

  MX_IWDG_Init();
  watchdog_initialized = true;
}

void bsp_watchdog_kick(void)
{
  // Support projects where IWDG is started outside this BSP module.
  if (!watchdog_initialized && (hiwdg.Instance == IWDG)) {
    watchdog_initialized = true;
  }

  if (!watchdog_initialized) {
    return;
  }

  (void)HAL_IWDG_Refresh(&hiwdg);
}
