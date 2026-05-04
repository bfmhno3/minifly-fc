// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  FreeRTOS application hook callbacks.
 *
 * @details
 * Provides hook functions called by the FreeRTOS kernel.  The idle hook
 * kicks the watchdog periodically and enters WFI to save power.
 */

#ifndef APP_HOOKS_H
#define APP_HOOKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  FreeRTOS idle task hook.
 *
 * Called repeatedly when no tasks are ready to run.  Kicks the hardware
 * watchdog at a fixed interval and issues WFI to enter low-power wait.
 * Configured via configUSE_IDLE_HOOK in FreeRTOSConfig.h.
 */
void app_idle_hook(void);

#ifdef __cplusplus
}
#endif

#endif
