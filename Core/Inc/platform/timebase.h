// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Platform-agnostic timebase abstraction.
 *
 * @details
 * Provides a stable millisecond-since-boot API decoupled from IRQ internals.
 * The actual tick source is owned by platform_irq.
 */

#ifndef PLATFORM_TIMEBASE_H
#define PLATFORM_TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the timebase.
 *
 * No-op kept for API symmetry -- the tick source is initialized
 * by platform_irq_init(), which runs first in platform_init().
 */
void platform_timebase_init(void);

/**
 * @brief  Get milliseconds since system start.
 *
 * @return Milliseconds since boot.
 */
uint32_t timebase_get_ms(void);

#ifdef __cplusplus
}
#endif

#endif
