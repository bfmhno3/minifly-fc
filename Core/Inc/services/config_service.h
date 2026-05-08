// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Configuration persistence service.
 *
 * @details
 * Manages the persistent config_param_t stored in internal flash.
 * Provides thread-safe read access via a mutex and deferred write-back
 * through a FreeRTOS queue.  The config_service_task blocks on the queue
 * and performs the actual flash erase/write, keeping flash operations
 * out of ISR and high-priority task contexts.
 */

#ifndef CONFIG_SERVICE_H
#define CONFIG_SERVICE_H

#include <stdbool.h>

#include "config_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the configuration service.
 *
 * Loads config from flash; falls back to compiled-in defaults on first boot
 * or if flash data is corrupt (version mismatch or bad checksum).
 * Creates the mutex and save queue.
 */
void config_service_init(void);

/**
 * @brief  Self-test: returns true if the service was initialized.
 */
bool config_service_test(void);

/**
 * @brief  Check whether the vehicle is currently flying.
 *
 * Used by pm_service to select the correct low-voltage threshold.
 */
bool config_service_is_flying(void);

/**
 * @brief  Set the flying state.
 *
 * @param[in] flying  true when airborne, false when landed.
 */
void config_service_set_flying(bool flying);

/**
 * @brief  Get a read-only pointer to the current config (mutex-protected).
 *
 * @return Pointer to config_param_t, or NULL if service is not initialized.
 */
const config_param_t *config_service_get(void);

/**
 * @brief  Get a mutable pointer to the config (caller must call mark_dirty).
 *
 * @warning NOT thread-safe -- the caller is responsible for not racing
 *          with config_service_get() or config_service_task().
 *          Intended for use in a single commander task context.
 *
 * @return Pointer to config_param_t, or NULL if service is not initialized.
 */
config_param_t *config_service_mut(void);

/**
 * @brief  Flag the config as modified and enqueue a flash save.
 *
 * Safe to call from task context or ISR (uses xQueueSendFromISR).
 */
void config_service_mark_dirty(void);

/**
 * @brief  Reset all PID gains to compiled-in defaults.
 *
 * Acquires the mutex, copies defaults for pidAngle/pidRate/pidPos,
 * then marks the config dirty to trigger a flash save.
 */
void config_service_reset_pid(void);

/**
 * @brief  FreeRTOS task entry -- deferred flash writer.
 *
 * Blocks on save_queue.  When a dirty notification arrives, acquires the
 * mutex and writes config_param_t to flash via bsp_flash.
 *
 * @param[in] arg  Unused.
 */
void config_service_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif