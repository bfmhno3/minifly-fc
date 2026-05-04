// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Application FreeRTOS task creation.
 *
 * @details
 * Creates all application-level FreeRTOS tasks using a table-driven
 * approach.  Called once from the FreeRTOS default task after the
 * scheduler has started.
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Create all application tasks.
 *
 * Iterates over a static table of task definitions and creates each one
 * via osThreadNew().  Calls Error_Handler() if any task creation fails.
 * Called once from the FreeRTOS default task entry in freertos.c.
 */
void app_tasks_create(void);

#ifdef __cplusplus
}
#endif

#endif
