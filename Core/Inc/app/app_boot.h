// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Application boot sequence.
 *
 * @details
 * Initializes board-level peripherals (LEDs, module connector, sensors)
 * before the FreeRTOS scheduler starts.  Called once from main() in the
 * USER CODE section.
 */

#ifndef APP_BOOT_H
#define APP_BOOT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize all board-level peripherals required before the
 *         scheduler starts.
 *
 * Currently initializes: BSP LED, BSP module connector, BSP sensors.
 */
void app_boot_init(void);

#ifdef __cplusplus
}
#endif

#endif
