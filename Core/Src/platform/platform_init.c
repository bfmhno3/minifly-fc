#include "platform/platform_init.h"

#include "bsp_led.h"
#include "bsp_module.h"
#include "bsp_sensors.h"
#include "services/config_service.h"
#include "control/stabilizer.h"
#include "modules/module_manager.h"

void platform_init(void)
{
    bsp_led_init();
    bsp_module_init();
    bsp_sensors_init();
    config_service_init();
    stabilizer_init();
    module_manager_init();
}

bool platform_self_test(void)
{
    return bsp_sensors_is_initialized();
}
