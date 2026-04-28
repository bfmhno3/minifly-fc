#include "app/app_tasks.h"

#include "main.h"
#include "cmsis_os.h"
#include "comm/radiolink.h"
#include "comm/usblink.h"
#include "comm/atkp.h"
#include "services/sensors.h"

#define APP_TASK_STACK_SMALL_BYTES (128U * 4U)
#define APP_TASK_STACK_MEDIUM_BYTES (192U * 4U)
#define APP_TASK_STACK_LARGE_BYTES (256U * 4U)
#define APP_TASK_PLACEHOLDER_DELAY_MS 1000U

typedef struct {
    osThreadId_t *handle;
    const char *name;
    uint32_t stack_size;
    osPriority_t priority;
    osThreadFunc_t func;
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
        {&stabilizer_task_handle, "stabilizer", APP_TASK_STACK_LARGE_BYTES, osPriorityHigh, app_task_placeholder_entry},
        {&module_manager_task_handle, "moduleMgr", APP_TASK_STACK_SMALL_BYTES, osPriorityLow, app_task_placeholder_entry},
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
