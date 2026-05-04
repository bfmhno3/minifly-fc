// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Application FreeRTOS task creation implementation.
 *
 * @details
 * Uses a table-driven approach: a static array of app_task_definition_t
 * entries defines each task's name, stack size, priority, and entry
 * function.  A single loop creates them all via osThreadNew().
 *
 * Placeholder entries (configSvc, pmSvc) use app_task_placeholder_entry
 * until their real implementations are wired up.
 */

#include "app/app_tasks.h"

#include "main.h"
#include "cmsis_os.h"
#include "comm/radiolink.h"
#include "comm/usblink.h"
#include "comm/atkp.h"
#include "services/sensors.h"
#include "control/stabilizer.h"
#include "modules/module_manager.h"

#define APP_TASK_STACK_SMALL_BYTES  (128U * 4U)   /**< 512 bytes */
#define APP_TASK_STACK_MEDIUM_BYTES (192U * 4U)   /**< 768 bytes */
#define APP_TASK_STACK_LARGE_BYTES  (256U * 4U)   /**< 1024 bytes */
#define APP_TASK_PLACEHOLDER_DELAY_MS 1000U       /**< placeholder task sleep interval */

/** @brief  Table entry describing a single FreeRTOS task. */
typedef struct {
    osThreadId_t *handle;       /**< receives the task handle from osThreadNew */
    const char *name;           /**< task name (max 16 chars for FreeRTOS) */
    uint32_t stack_size;        /**< stack size in bytes */
    osPriority_t priority;      /**< CMSIS-RTOS priority */
    osThreadFunc_t func;        /**< task entry function */
} app_task_definition_t;

static osThreadId_t radiolink_task_handle;
static osThreadId_t usblink_rx_task_handle;
static osThreadId_t usblink_tx_task_handle;
static osThreadId_t atkp_tx_task_handle;
static osThreadId_t atkp_rx_task_handle;
static osThreadId_t config_service_task_handle;
static osThreadId_t pm_service_task_handle;
static osThreadId_t sensors_task_handle;
static osThreadId_t stabilizer_task_handle;
static osThreadId_t module_manager_task_handle;

/**
 * @brief  Placeholder task entry for services not yet implemented.
 *
 * Simply sleeps in a loop.  Replaced with the real entry function
 * once the service is ready.
 */
static void app_task_placeholder_entry(void *argument)
{
    (void)argument;

    for (;;) {
        osDelay(APP_TASK_PLACEHOLDER_DELAY_MS);
    }
}

void app_tasks_create(void)
{
    static const app_task_definition_t task_definitions[] = {
        {&radiolink_task_handle, "radiolink", APP_TASK_STACK_SMALL_BYTES, osPriorityHigh, radiolink_task},
        {&usblink_rx_task_handle, "usblinkRx", APP_TASK_STACK_SMALL_BYTES, osPriorityAboveNormal, usblink_rx_task},
        {&usblink_tx_task_handle, "usblinkTx", APP_TASK_STACK_SMALL_BYTES, osPriorityNormal, usblink_tx_task},
        {&atkp_tx_task_handle, "atkpTx", APP_TASK_STACK_SMALL_BYTES, osPriorityNormal, atkp_tx_task},
        {&atkp_rx_task_handle, "atkpRx", APP_TASK_STACK_MEDIUM_BYTES, osPriorityHigh, atkp_rx_task},
        {&config_service_task_handle, "configSvc", APP_TASK_STACK_SMALL_BYTES, osPriorityLow, app_task_placeholder_entry},
        {&pm_service_task_handle, "pmSvc", APP_TASK_STACK_SMALL_BYTES, osPriorityBelowNormal, app_task_placeholder_entry},
        {&sensors_task_handle, "sensors", APP_TASK_STACK_LARGE_BYTES, osPriorityAboveNormal, sensors_task},
        {&stabilizer_task_handle, "stabilizer", APP_TASK_STACK_LARGE_BYTES, osPriorityHigh, stabilizer_task},
        {&module_manager_task_handle, "moduleMgr", APP_TASK_STACK_SMALL_BYTES, osPriorityLow, module_manager_task},
    };

    for (uint32_t index = 0; index < (sizeof(task_definitions) / sizeof(task_definitions[0])); ++index) {
        const app_task_definition_t *task_definition = &task_definitions[index];
        const osThreadAttr_t attributes = {
            .name = task_definition->name,
            .stack_size = task_definition->stack_size,
            .priority = task_definition->priority,
        };

        *task_definition->handle = osThreadNew(task_definition->func, NULL, &attributes);
        if (*task_definition->handle == NULL) {
            Error_Handler();
        }
    }
}
