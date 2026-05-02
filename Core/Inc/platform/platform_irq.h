#ifndef PLATFORM_IRQ_H
#define PLATFORM_IRQ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void platform_irq_init(void);
uint32_t platform_irq_get_tick_ms(void);

#ifdef __cplusplus
}
#endif

#endif
