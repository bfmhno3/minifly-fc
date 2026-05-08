// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  IRQ configuration and system tick management.
 *
 * @details
 * Cortex-M4 specific: relocates the vector table via SCB->VTOR,
 * sets NVIC priority grouping for FreeRTOS compatibility, and
 * provides the SysTick-based millisecond tick.
 */

#include "platform/platform_irq.h"

#include "board.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "services/sensors.h"
#include "comm/radiolink.h"

// extern void xPortSysTickHandler(void);

/* Accumulates SysTick interrupts before the FreeRTOS scheduler starts.
 * Unused once xTaskGetTickCount() becomes the authoritative tick source. */
static volatile uint32_t irq_tick_cnt;

void platform_irq_init(void)
{
    /* Relocate vector table to app offset -- bootloader occupies the start of flash */
    SCB->VTOR = FLASH_BASE | BOARD_FLASH_APP_OFFSET;
    /* Group 4: 4 bits pre-emption, 0 bits sub-priority -- matches FreeRTOS configPRIO_BITS */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

uint32_t platform_irq_get_tick_ms(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        return xTaskGetTickCount();
    return irq_tick_cnt;
}

/**
 * @brief  Cortex-M system tick ISR.
 *
 * Delegates to FreeRTOS tick handler once the scheduler is running;
 * otherwise increments the bare-metal counter for pre-scheduler delays.
 */
// void SysTick_Handler(void)
// {
//     if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
//         xPortSysTickHandler();
//     else
//         irq_tick_cnt++;
// }

/**
 * @brief  Unified GPIO EXTI callback -- dispatches to per-module handlers.
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    sensors_exti_callback(pin);
    radiolink_exti_callback(pin);
}
