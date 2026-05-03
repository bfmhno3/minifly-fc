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
