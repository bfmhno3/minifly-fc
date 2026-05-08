// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Expansion module lifecycle manager.
 *
 * @details
 * Detects which expansion module (LED ring, WiFi camera, optical flow) is
 * plugged into the shared module connector via an ADC identification pin.
 * Handles debounced hot-swap: when a new module is confirmed, the old one
 * is de-initialized and the new one is started.
 */

#ifndef MODULES_MODULE_MANAGER_H
#define MODULES_MODULE_MANAGER_H

#include "bsp_module.h"

/**
 * @brief  Module manager state (unused externally, reserved for diagnostics).
 */
typedef enum module_mgr_state {
  MODULE_MGR_STATE_IDLE = 0,  /**< No module detected. */
  MODULE_MGR_STATE_DETECTING, /**< Debounce in progress. */
  MODULE_MGR_STATE_ACTIVE,    /**< Module confirmed and running. */
} module_mgr_state_t;

/**
 * @brief  Initialize the module manager.
 *
 * Resets internal state to "no module active".  Call once before starting
 * the module manager task.
 */
void module_manager_init(void);

/**
 * @brief  FreeRTOS task entry -- periodic module detection loop.
 *
 * Reads the module ADC pin every MODULE_DETECT_PERIOD_MS, debounces the
 * reading, and triggers init/deinit transitions when the detected module
 * changes.  Runs forever.
 *
 * @param[in] arg  Unused.
 */
void module_manager_task(void *arg);

/**
 * @brief  Get the currently active module ID.
 *
 * @return bsp_module_id_t of the active module, or BSP_MODULE_NONE.
 */
bsp_module_id_t module_manager_get_active(void);

/**
 * @brief  Manually control the shared module power pin.
 *
 * @param[in] on  true to enable power, false to disable.
 */
void module_manager_power_enable(bool on);

#endif /* MODULES_MODULE_MANAGER_H */
