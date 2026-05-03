// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Platform fault handling and Cortex-M exception vectors.
 *
 * @details
 * Implements the fault panic path and Cortex-M exception handlers.
 * The busy-wait LED blink has no timer dependency, so it works even
 * with all interrupts disabled.
 *
 * Hardware dependencies: BSP motors (bsp_motors.h), BSP LEDs (bsp_led.h).
 */

#include "platform/platform_fault.h"

#include "stm32f4xx_hal.h"
#include "bsp_motors.h"
#include "bsp_led.h"

/* Busy-wait loop count calibrated for visible LED blink at system clock frequency.
 * Not a time unit -- actual duration depends on clock speed and compiler optimization. */
#define FAULT_BLINK_DELAY 400000U

/**
 * @brief  Toggle red LED with a busy-wait delay.
 *
 * No timer dependency -- works even with all interrupts disabled.
 */
static void fault_led_blink(void)
{
	bsp_led_toggle(BSP_LED_RED_L);
	for (volatile uint32_t i = 0; i < FAULT_BLINK_DELAY; i++)
		__NOP();
}

void platform_fault_init(void)
{
	/* Intentionally empty -- reserved for future fault-reporting hooks
	 * (e.g., writing fault code to backup register or flash). */
}

void platform_fault_shutdown(void)
{
	bsp_motors_stop_all();

	for (uint8_t i = 0; i < BSP_LED_COUNT; i++)
		bsp_led_set(i, false);
}

/**
 * @brief  Enter a permanent fault state.
 *
 * Disables all interrupts, brings the system to a safe state, then blinks
 * the red LED indefinitely. Never returns.
 *
 * @warning All interrupts are disabled -- the system is fully halted.
 */
void platform_fault_panic(uint32_t code)
{
	(void)code; /* Reserved for future use (e.g., blink-pattern encoding by fault type) */

	__disable_irq();
	platform_fault_shutdown();

	while (1)
		fault_led_blink();
}

/* Cortex-M exception vectors -- invoked by hardware on fault conditions. */

void HardFault_Handler(void)
{
	platform_fault_panic(PLATFORM_FAULT_HARD);
}

void MemManage_Handler(void)
{
	platform_fault_panic(PLATFORM_FAULT_MEM);
}

void BusFault_Handler(void)
{
	platform_fault_panic(PLATFORM_FAULT_BUS);
}

void UsageFault_Handler(void)
{
	platform_fault_panic(PLATFORM_FAULT_USAGE);
}
