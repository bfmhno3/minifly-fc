// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  FreeRTOS application hook callbacks implementation.
 *
 * @details
 * The idle hook runs a watchdog kick at a fixed interval (100 OS ticks)
 * and then issues WFI (Wait For Interrupt) to reduce power consumption
 * when no tasks are ready.
 */

#include "app/app_hooks.h"

#include "bsp_watchdog.h"
#include "cmsis_os2.h"
#include "main.h"

#define APP_IDLE_WATCHDOG_KICK_INTERVAL_TICKS 100U  /**< watchdog kick period in OS ticks */

void app_idle_hook(void)
{
    static uint32_t last_watchdog_kick_tick;
    const uint32_t current_tick = osKernelGetTickCount();

    if ((current_tick - last_watchdog_kick_tick) >= APP_IDLE_WATCHDOG_KICK_INTERVAL_TICKS) {
        last_watchdog_kick_tick = current_tick;
        bsp_watchdog_kick();
    }

    /* WFI: enter low-power wait until the next interrupt (systick or peripheral) */
    __WFI();
}
