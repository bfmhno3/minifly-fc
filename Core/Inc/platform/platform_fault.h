#ifndef PLATFORM_FAULT_H
#define PLATFORM_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum platform_fault_code {
	PLATFORM_FAULT_HARD   = 0,
	PLATFORM_FAULT_MEM    = 1,
	PLATFORM_FAULT_BUS    = 2,
	PLATFORM_FAULT_USAGE  = 3,
	PLATFORM_FAULT_ASSERT = 4,
};

void platform_fault_init(void);
void platform_fault_shutdown(void);
void platform_fault_panic(uint32_t code);

#ifdef __cplusplus
}
#endif

#endif
