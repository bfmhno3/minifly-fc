// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Platform initialization sequence.
 *
 * @details
 * Orchestrates boot-time init of all platform subsystems. The call order
 * is intentional: IRQ and timebase must be up before anything that uses
 * delays or timeouts.
 */

#include "platform/platform_init.h"

#include "platform/platform_irq.h"
#include "platform/platform_fault.h"
#include "platform/timebase.h"
#include "bsp_led.h"
#include "bsp_module.h"
#include "bsp_sensors.h"
#include "services/config_service.h"
#include "control/stabilizer.h"
#include "modules/module_manager.h"

void platform_init(void)
{
  /* IRQ and timebase first -- anything that uses delays or timeouts
     * depends on the tick source being available. */
  platform_irq_init();
  platform_timebase_init();
  platform_fault_init();
  bsp_led_init();
  bsp_module_init();
  bsp_sensors_init();
  config_service_init();
  stabilizer_init();
  module_manager_init();
}

/**
 * @brief  Post-init sanity check.
 *
 * Currently only verifies sensor initialization status.
 */
bool platform_self_test(void)
{
  return bsp_sensors_is_initialized();
}
