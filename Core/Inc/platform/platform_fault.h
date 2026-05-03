// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Platform fault and panic API.
 *
 * @details
 * Provides a unified fault-handling entry point for Cortex-M hardware exceptions
 * (HardFault, MemManage, BusFault, UsageFault) and software assertions.
 * All fault paths disable interrupts and halt the system in a safe state.
 */

#ifndef PLATFORM_FAULT_H
#define PLATFORM_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Fault codes mapping to Cortex-M exception types.
 *
 * Passed to platform_fault_panic() to identify the source of the fault.
 */
enum platform_fault_code {
	PLATFORM_FAULT_HARD   = 0,
	PLATFORM_FAULT_MEM    = 1,
	PLATFORM_FAULT_BUS    = 2,
	PLATFORM_FAULT_USAGE  = 3,
	PLATFORM_FAULT_ASSERT = 4,
};

/**
 * @brief  Initialize fault handling subsystem.
 *
 * Currently a no-op placeholder reserved for future fault-reporting hooks
 * (e.g., backup register or flash logging).
 */
void platform_fault_init(void);

/**
 * @brief  Bring the system to a safe state.
 *
 * Stops all motors and turns off all LEDs. Called internally by
 * platform_fault_panic() before halting.
 */
void platform_fault_shutdown(void);

/**
 * @brief  Enter a permanent fault state.
 *
 * Disables all interrupts, calls platform_fault_shutdown(), then blinks
 * the red LED in an infinite loop. This function never returns.
 *
 * @param[in] code  Fault code identifying the exception source.
 *
 * @warning Disables all interrupts -- the system is fully halted after this call.
 */
void platform_fault_panic(uint32_t code);

#ifdef __cplusplus
}
#endif

#endif
