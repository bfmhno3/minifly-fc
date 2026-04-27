#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

typedef struct {
    uint8_t ctrlMode   : 2;
    uint8_t keyFlight  : 1;
    uint8_t keyLand    : 1;
    uint8_t emerStop   : 1;
    uint8_t flightMode : 1;
    uint8_t reserved   : 2;
} commander_bits_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float trim_pitch;
    float trim_roll;
    uint16_t thrust;
} ctrl_val_t;

typedef struct {
    ctrl_val_t buf[2];
    volatile bool active_side;
    uint32_t timestamp;
} ctrl_cache_t;

typedef enum {
    CTRL_SRC_REMOTER = 0,
    CTRL_SRC_WIFI    = 1,
} ctrl_src_t;

typedef enum {
    RPY_RATE  = 0,
    RPY_ANGLE = 1,
} rpy_type_t;

typedef enum {
    YAW_XMODE    = 0,
    YAW_CAREFREE = 1,
} yaw_mode_t;

/* Lifecycle */
void commander_init(void);

/* Main entry: called at 100Hz from stabilizerTask */
void commander_get_setpoint(setpoint_t *sp, const state_t *state);

/* Write raw control data from producer tasks (radiolink / USB) */
void commander_cache_ctrl_data(ctrl_src_t src, const ctrl_val_t *val);

/* Query current state */
uint8_t commander_get_ctrl_mode(void);
bool commander_get_key_flight(void);
bool commander_get_key_land(void);
bool commander_get_emer_stop(void);

/* Set flight state bits (called from command handlers) */
void commander_set_ctrl_mode(uint8_t mode);
void commander_set_key_flight(bool set);
void commander_set_key_land(bool set);
void commander_set_flight_mode(bool carefree);
void commander_set_emer_stop(bool set);

#ifdef __cplusplus
}
#endif

#endif /* __COMMANDER_H */
