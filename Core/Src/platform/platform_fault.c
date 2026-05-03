#include "platform/platform_fault.h"

#include "stm32f4xx_hal.h"
#include "bsp_motors.h"
#include "bsp_led.h"

#define FAULT_BLINK_DELAY 400000U

static void fault_led_blink(void)
{
	bsp_led_toggle(BSP_LED_RED_L);
	for (volatile uint32_t i = 0; i < FAULT_BLINK_DELAY; i++)
		__NOP();
}

void platform_fault_init(void)
{
}

void platform_fault_shutdown(void)
{
	bsp_motors_stop_all();

	for (uint8_t i = 0; i < BSP_LED_COUNT; i++)
		bsp_led_set(i, false);
}

void platform_fault_panic(uint32_t code)
{
	(void)code;

	__disable_irq();
	platform_fault_shutdown();

	while (1)
		fault_led_blink();
}

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
