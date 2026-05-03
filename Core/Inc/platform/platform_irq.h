// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Low-level IRQ and vector table configuration.
 *
 * @details
 * Relocates the Cortex-M vector table to the application flash offset
 * (for bootloader coexistence) and configures NVIC priority grouping.
 * Also provides a millisecond tick source that works both before and
 * after the FreeRTOS scheduler has started.
 */

#ifndef PLATFORM_IRQ_H
#define PLATFORM_IRQ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Relocate vector table and configure NVIC priority grouping.
 *
 * @pre    Must be called before any interrupt-driven module.
 */
void platform_irq_init(void);

/**
 * @brief  Get milliseconds since system boot.
 *
 * Uses FreeRTOS tick count after the scheduler starts; falls back to
 * a bare-metal SysTick counter before that.
 *
 * @return Milliseconds since boot.
 */
uint32_t platform_irq_get_tick_ms(void);

#ifdef __cplusplus
}
#endif

#endif
