#include "services/config_service.h"

#include <string.h>
#include "bsp_flash.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define CONFIG_PARAM_ADDR (0x08004000UL)

static config_param_t config_param;
static config_param_t config_param_default;

static bool is_init = false;
static bool is_dirty = false;
static volatile bool is_flying = false;

static SemaphoreHandle_t mutex = NULL;
static QueueHandle_t save_queue = NULL;

static uint8_t config_checksum(const config_param_t *data)
{
    const uint8_t *c = (const uint8_t *)data;
    size_t len = sizeof(config_param_t);
    uint8_t sum = 0;

    for (size_t i = 0; i < len; i++)
    {
        sum += *(c++);
    }
    sum -= data->checksum;

    return sum;
}

static void config_set_defaults(void)
{
    config_param.version = CONFIG_VERSION;

    config_param.pidAngle.roll.kp = 8.0f;
    config_param.pidAngle.roll.ki = 0.0f;
    config_param.pidAngle.roll.kd = 0.0f;

    config_param.pidAngle.pitch.kp = 8.0f;
    config_param.pidAngle.pitch.ki = 0.0f;
    config_param.pidAngle.pitch.kd = 0.0f;

    config_param.pidAngle.yaw.kp = 20.0f;
    config_param.pidAngle.yaw.ki = 0.0f;
    config_param.pidAngle.yaw.kd = 1.5f;

    config_param.pidRate.roll.kp = 300.0f;
    config_param.pidRate.roll.ki = 0.0f;
    config_param.pidRate.roll.kd = 6.5f;

    config_param.pidRate.pitch.kp = 300.0f;
    config_param.pidRate.pitch.ki = 0.0f;
    config_param.pidRate.pitch.kd = 6.5f;

    config_param.pidRate.yaw.kp = 200.0f;
    config_param.pidRate.yaw.ki = 18.5f;
    config_param.pidRate.yaw.kd = 0.0f;

    config_param.pidPos.vx.kp = 4.5f;
    config_param.pidPos.vx.ki = 0.0f;
    config_param.pidPos.vx.kd = 0.0f;

    config_param.pidPos.vy.kp = 4.5f;
    config_param.pidPos.vy.ki = 0.0f;
    config_param.pidPos.vy.kd = 0.0f;

    config_param.pidPos.vz.kp = 100.0f;
    config_param.pidPos.vz.ki = 150.0f;
    config_param.pidPos.vz.kd = 10.0f;

    config_param.pidPos.x.kp = 4.0f;
    config_param.pidPos.x.ki = 0.0f;
    config_param.pidPos.x.kd = 0.6f;

    config_param.pidPos.y.kp = 4.0f;
    config_param.pidPos.y.ki = 0.0f;
    config_param.pidPos.y.kd = 0.6f;

    config_param.pidPos.z.kp = 6.0f;
    config_param.pidPos.z.ki = 0.0f;
    config_param.pidPos.z.kd = 4.5f;

    config_param.trimP = 0.0f;
    config_param.trimR = 0.0f;
    config_param.thrustBase = 34000;

    config_param.cmdTune.rateScaleRP = 360.0f;
    config_param.cmdTune.rateScaleYaw = 180.0f;
    config_param.cmdTune.angleScaleRP = 30.0f;
    config_param.cmdTune.yawRateScale = 120.0f;
    config_param.cmdTune.autolandDescent = 0.5f;
    config_param.cmdTune.autolandRampStep = 327.675f;
    config_param.cmdTune.takeoffRampStep = 655.35f;
    config_param.cmdTune.takeoffMinThrust = 16383.75f;

    config_param.checksum = config_checksum(&config_param);

    memcpy(&config_param_default, &config_param, sizeof(config_param_t));
}

static bool config_load_from_flash(void)
{
    if (!bsp_flash_read(CONFIG_PARAM_ADDR, &config_param, sizeof(config_param_t)))
    {
        return false;
    }

    if (config_param.version == CONFIG_VERSION)
    {
        if (config_checksum(&config_param) == config_param.checksum)
        {
            return true;
        }
    }

    return false;
}

static bool config_save_to_flash(void)
{
    config_param.checksum = config_checksum(&config_param);

    if (!bsp_flash_erase(CONFIG_PARAM_ADDR, sizeof(config_param_t)))
    {
        return false;
    }

    return bsp_flash_write(CONFIG_PARAM_ADDR, &config_param, sizeof(config_param_t));
}

void config_service_init(void)
{
    if (is_init)
    {
        return;
    }

    config_set_defaults();

    if (!config_load_from_flash())
    {
        config_save_to_flash();
    }

    memcpy(&config_param_default, &config_param, sizeof(config_param_t));

    mutex = xSemaphoreCreateMutex();
    save_queue = xQueueCreate(4, sizeof(bool));

    is_init = true;
}

bool config_service_test(void)
{
    return is_init;
}

bool config_service_is_flying(void)
{
    return is_flying;
}

void config_service_set_flying(bool flying)
{
    is_flying = flying;
}

const config_param_t* config_service_get(void)
{
    if (!is_init)
    {
        return NULL;
    }

    const config_param_t *result;
    xSemaphoreTake(mutex, portMAX_DELAY);
    result = &config_param;
    xSemaphoreGive(mutex);

    return result;
}

config_param_t* config_service_mut(void)
{
    if (!is_init)
    {
        return NULL;
    }

    return &config_param;
}

void config_service_mark_dirty(void)
{
    if (!is_init)
    {
        return;
    }

    is_dirty = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(save_queue, &is_dirty, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void config_service_reset_pid(void)
{
    if (!is_init)
    {
        return;
    }

    xSemaphoreTake(mutex, portMAX_DELAY);
    config_param.pidAngle = config_param_default.pidAngle;
    config_param.pidRate = config_param_default.pidRate;
    config_param.pidPos = config_param_default.pidPos;
    xSemaphoreGive(mutex);

    config_service_mark_dirty();
}

void config_service_task(void *arg)
{
    (void)arg;

    bool dirty = false;

    while (1)
    {
        if (xQueueReceive(save_queue, &dirty, portMAX_DELAY) == pdTRUE)
        {
            if (dirty)
            {
                xSemaphoreTake(mutex, portMAX_DELAY);
                bool success = config_save_to_flash();
                xSemaphoreGive(mutex);

                if (success)
                {
                    is_dirty = false;
                }
            }
        }
    }
}