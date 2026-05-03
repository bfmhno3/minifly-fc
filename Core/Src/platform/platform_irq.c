#include "platform/platform_irq.h"

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * Firmware base offset for the vector table.
 * Must match the FLASH origin in the linker script.
 * Update this when the bootloader/config area layout changes.
 */
#define FIRMWARE_BASE_OFFSET 0x0U

static volatile uint32_t irq_tick_cnt;

void platform_irq_init(void)
{
    SCB->VTOR = FLASH_BASE | FIRMWARE_BASE_OFFSET;
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

uint32_t platform_irq_get_tick_ms(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        return xTaskGetTickCount();
    return irq_tick_cnt;
}

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        xPortSysTickHandler();
    else
        irq_tick_cnt++;
}
