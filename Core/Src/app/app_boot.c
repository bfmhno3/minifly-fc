// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Application boot sequence implementation.
 *
 * @details
 * Calls BSP init functions in a specific order: LEDs first (for visual
 * feedback), then the module connector, then sensors.  All calls happen
 * before the FreeRTOS scheduler starts, so blocking is acceptable.
 */

#include "app/app_boot.h"

#include "platform/platform_init.h"

void app_boot_init(void)
{
  platform_init();
}
