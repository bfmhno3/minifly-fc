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

#include "bsp_led.h"
#include "bsp_module.h"
#include "bsp_sensors.h"

void app_boot_init(void)
{
    bsp_led_init();       /* first: visual feedback for subsequent init */
    bsp_module_init();    /* module connector ADC and power pin */
    bsp_sensors_init();   /* IMU, barometer, etc. */
}
