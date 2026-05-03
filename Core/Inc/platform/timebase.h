#ifndef PLATFORM_TIMEBASE_H
#define PLATFORM_TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void platform_timebase_init(void);
uint32_t timebase_get_ms(void);

#ifdef __cplusplus
}
#endif

#endif
