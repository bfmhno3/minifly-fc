// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Command interpreter -- translates raw RC/WiFi control data into
 *         flight setpoints for the stabilizer.
 *
 * @details
 * Consumes control data from radiolink and usblink via commander_cache_ctrl_data().
 * The stabilizerTask calls commander_get_setpoint() at 100 Hz to obtain the
 * current setpoint.  Internal logic handles:
 * - Source selection (remote vs WiFi, newest timestamp wins)
 * - Low-pass filtering on all axes
 * - Watchdog timeout (stabilize -> auto-land -> shutdown)
 * - Mode dispatch (rate / angle / full-assist)
 * - Carefree yaw rotation
 * - One-key takeoff ramp and auto-land descent
 *
 * All public API is safe to call from task context only.
 */

#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Watchdog: hold position after this many ms without new control data. */
#define COMMANDER_WDT_TIMEOUT_STABILIZE  500

/** @brief Watchdog: initiate auto-land after this many ms without new control data. */
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

/**
 * @brief  Bitfield for flight control flags.
 *
 * Packed into a single byte for efficient queue transfer.
 */
typedef struct {
    uint8_t ctrlMode   : 2; /**< 0=rate, 1=angle, 2=full-assist */
    uint8_t keyFlight  : 1; /**< 1 = takeoff commanded */
    uint8_t keyLand    : 1; /**< 1 = land commanded */
    uint8_t emerStop   : 1; /**< 1 = emergency stop active */
    uint8_t flightMode : 1; /**< 0=X-mode, 1=carefree */
    uint8_t reserved   : 2;
} commander_bits_t;

/**
 * @brief  Raw control values from one source (remote or WiFi).
 */
typedef struct {
    float roll;          /**< Roll stick (-1.0 .. 1.0) */
    float pitch;         /**< Pitch stick (-1.0 .. 1.0) */
    float yaw;           /**< Yaw stick (-1.0 .. 1.0) */
    float trim_pitch;    /**< Pitch trim offset */
    float trim_roll;     /**< Roll trim offset */
    uint16_t thrust;     /**< Thrust stick (0 .. 65535) */
} ctrl_val_t;

/**
 * @brief  Double-buffered control cache for lock-free producer/consumer.
 *
 * The producer writes to the inactive side then flips active_side.
 * The consumer reads from the active side.
 */
typedef struct {
    ctrl_val_t buf[2];
    volatile bool active_side;
    uint32_t timestamp;
} ctrl_cache_t;

/** @brief Control data source. */
typedef enum {
    CTRL_SRC_REMOTER = 0, /**< nRF24L01 radio link */
    CTRL_SRC_WIFI    = 1, /**< WiFi module */
} ctrl_src_t;

/** @brief Rate vs. angle mode for roll/pitch. */
typedef enum {
    RPY_RATE  = 0, /**< Angular rate command */
    RPY_ANGLE = 1, /**< Absolute angle command */
} rpy_type_t;

/** @brief Yaw reference frame. */
typedef enum {
    YAW_XMODE    = 0, /**< Body-frame yaw */
    YAW_CAREFREE = 1, /**< World-frame yaw (heading hold) */
} yaw_mode_t;

/** @brief  Initialize commander (loads tuning config, zeros state). */
void commander_init(void);

/**
 * @brief  Compute the current flight setpoint from cached control data.
 *
 * Called by stabilizerTask at 100 Hz.  Applies source selection, watchdog,
 * LPF, mode dispatch, takeoff/land ramps, and writes the result into @p sp.
 *
 * @param[out] sp     Destination setpoint.
 * @param[in]  state  Current attitude state (used for carefree rotation).
 */
void commander_get_setpoint(setpoint_t *sp, const state_t *state);

/**
 * @brief  Cache raw control data from a producer task.
 *
 * Uses double-buffering with a memory fence so the consumer never sees
 * a torn read.  Safe to call from any task (radio RX, WiFi RX).
 *
 * @param[in] src  Which link the data came from.
 * @param[in] val  Raw stick values.
 */
void commander_cache_ctrl_data(ctrl_src_t src, const ctrl_val_t *val);

/** @brief  Get current control mode (0=rate, 1=angle, 2=full-assist). */
uint8_t commander_get_ctrl_mode(void);

/** @brief  Get takeoff key state. */
bool commander_get_key_flight(void);

/** @brief  Get land key state. */
bool commander_get_key_land(void);

/** @brief  Get emergency stop state. */
bool commander_get_emer_stop(void);

/**
 * @brief  Set control mode.
 *
 * @param[in] mode  0=rate, 1=angle, 2=full-assist (masked to 2 bits).
 */
void commander_set_ctrl_mode(uint8_t mode);

/**
 * @brief  Set/clear takeoff key and arm the takeoff ramp.
 *
 * @param[in] set  true to arm, false to cancel.
 */
void commander_set_key_flight(bool set);

/**
 * @brief  Set/clear land key and arm the auto-land sequence.
 *
 * @param[in] set  true to arm, false to cancel.
 */
void commander_set_key_land(bool set);

/**
 * @brief  Select yaw reference frame.
 *
 * @param[in] carefree  true for world-frame, false for body-frame.
 */
void commander_set_flight_mode(bool carefree);

/**
 * @brief  Set emergency stop (motors off immediately).
 *
 * @param[in] set  true to activate, false to clear.
 */
void commander_set_emer_stop(bool set);

#ifdef __cplusplus
}
#endif

#endif /* __COMMANDER_H */
