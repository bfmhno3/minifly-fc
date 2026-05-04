// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  WiFi RC receiver expansion module.
 *
 * @details
 * Receives RC commands from a WiFi-based remote controller over UART1.
 * A dedicated FreeRTOS task parses a simple framed protocol (start byte,
 * 6 data bytes, XOR checksum, end byte) and translates stick values into
 * the commander control interface.
 *
 * Hardware dependencies:
 * - USART1 for WiFi receiver communication.
 */

#ifndef MODULES_WIFI_MODULE_H
#define MODULES_WIFI_MODULE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  Initialize the WiFi module.
 *
 * Creates the UART receive task (or resumes it on re-init).
 * Safe to call multiple times.
 */
void wifi_module_init(void);

/**
 * @brief  De-initialize the WiFi module.
 *
 * Suspends the UART receive task (does not destroy it).
 */
void wifi_module_deinit(void);

#endif /* MODULES_WIFI_MODULE_H */
