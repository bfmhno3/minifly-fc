// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Timebase implementation.
 *
 * @details
 * Thin facade over platform_irq_get_tick_ms(), providing a stable time
 * API that does not expose IRQ-layer internals to upper layers.
 */

#include "platform/timebase.h"

#include "platform/platform_irq.h"

void platform_timebase_init(void)
{
  /* tick source is already initialized by platform_irq_init() */
}

uint32_t timebase_get_ms(void)
{
  return platform_irq_get_tick_ms();
}
