// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Communication stack facade.
 *
 * @details
 * Initializes the full communication subsystem in dependency order:
 * radiolink -> usblink -> atkp -> commander.  Provides a single
 * entry point for app_tasks and a self-test query.
 */

#ifndef __COMM_STACK_H
#define __COMM_STACK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the entire communication stack.
 *
 * Brings up radiolink, usblink, atkp, and commander in sequence.
 * Safe to call multiple times (subsequent calls are no-ops).
 */
void comm_stack_init(void);

/**
 * @brief  Self-test: returns true if comm_stack_init() has been called.
 */
bool comm_stack_test(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_STACK_H */
