// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Power management service.
 *
 * @details
 * Monitors battery voltage received via syslink packets and tracks
 * charging state based on charger flags.  Provides low-power detection
 * with hysteresis (different thresholds for flying vs. static).
 *
 * Voltage data arrives through the ATKP communication stack as
 * pm_syslink_info_t payloads.
 */

#ifndef PM_SERVICE_H
#define PM_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @name Low-voltage thresholds
 * @{ */
#define PM_BAT_LOW_VOLTAGE_FLY \
  3.00f /**< Low-voltage threshold when airborne (V per cell) */
#define PM_BAT_LOW_VOLTAGE_STATIC \
  3.60f /**< Low-voltage threshold when stationary (V per cell) */
#define PM_BAT_LOW_TIMEOUT_MS \
  5000 /**< Time below threshold before entering low-power (ms) */
#define PM_VOLTAGE_INVALID 0.0f /**< Sentinel: no valid voltage reading yet */
/** @} */

/**
 * @brief  Battery status payload received from syslink (packed for wire format).
 */
typedef struct {
  uint8_t flags; /**< Bit 0: power-good, Bit 1: charging active */
  float vBat;    /**< Battery voltage (V) */
} __attribute__((packed)) pm_syslink_info_t;

/**
 * @brief  Power management state machine states.
 */
typedef enum {
  PM_STATE_BATTERY = 0, /**< Running from battery, normal operation */
  PM_STATE_CHARGING,    /**< USB connected, actively charging */
  PM_STATE_CHARGED,     /**< USB connected, charge complete */
  PM_STATE_LOW_POWER,   /**< Battery critically low */
  PM_STATE_SHUT_DOWN,   /**< Shutdown imminent (reserved) */
} pm_state_t;

/**
 * @brief  Initialize power management service.
 *
 * Sets initial voltage to PM_VOLTAGE_INVALID and state to PM_STATE_BATTERY.
 */
void pm_service_init(void);

/**
 * @brief  Self-test: returns true if init completed and voltage is valid.
 */
bool pm_service_test(void);

/**
 * @brief  FreeRTOS task entry -- periodic power state evaluation.
 *
 * Checks battery voltage against thresholds, evaluates charger flags,
 * and transitions pm_state accordingly.
 *
 * @param[in] param  Unused.
 */
void pm_service_task(void *param);

/**
 * @brief  Update battery voltage from a received syslink ATKP packet.
 *
 * @param[in] atkp  Pointer to atkp_t containing pm_syslink_info_t payload.
 *
 * @warning Expects packet->data to hold at least sizeof(pm_syslink_info_t) bytes.
 */
void pm_service_update_voltage(void *atkp);

/**
 * @brief  Get the most recent battery voltage.
 *
 * @return Voltage in volts, or PM_VOLTAGE_INVALID if no reading yet.
 */
float pm_service_get_voltage(void);

/** @brief  Get the peak battery voltage seen since init. */
float pm_service_get_voltage_max(void);

/** @brief  Get the minimum battery voltage seen since init. */
float pm_service_get_voltage_min(void);

/** @brief  Get the current power management state. */
pm_state_t pm_service_get_state(void);

/** @brief  Returns true if the system is in low-power state. */
bool pm_service_is_low_power(void);

/** @brief  Returns true if the battery is currently charging. */
bool pm_service_is_charging(void);

#ifdef __cplusplus
}
#endif

#endif