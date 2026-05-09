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

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "stm32f4xx_hal.h"
#include "services/sensors.h"
#include "comm/radiolink.h"

// extern void xPortSysTickHandler(void);

/* Accumulates SysTick interrupts before the FreeRTOS scheduler starts.
 * Unused once xTaskGetTickCount() becomes the authoritative tick source. */
static volatile uint32_t irq_tick_cnt;

void platform_irq_init(void)
{
  /* Relocate vector table to app offset -- bootloader occupies the start of flash.
   * FIXME: CubeMX-generated HAL_Init() runs before this point and configures
   *        SysTick using the old VTOR.  If the bootloader does NOT set VTOR
   *        before jumping here, there is a brief window where the wrong vector
   *        table is active.  Verify bootloader behavior or move this call to
   *        the earliest possible USER CODE section in main.c. */
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

/*
 * SysTick_Handler is provided by cmsis_os2.c (USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=0).
 * The HAL timebase uses TIM1 (see stm32f4xx_hal_timebase_tim.c), so HAL_Delay()
 * works during platform_init() before the FreeRTOS scheduler starts.
 * irq_tick_cnt is unused in this configuration.
 */

/**
 * @brief  Unified GPIO EXTI callback -- dispatches to per-module handlers.
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  sensors_exti_callback(pin);
  radiolink_exti_callback(pin);
}
