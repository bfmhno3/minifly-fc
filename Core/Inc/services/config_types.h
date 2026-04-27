#ifndef CONFIG_TYPES_H
#define CONFIG_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_VERSION 13

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_param_t;

typedef struct
{
    pid_param_t roll;
    pid_param_t pitch;
    pid_param_t yaw;
} pid_group_t;

typedef struct
{
    pid_param_t vx;
    pid_param_t vy;
    pid_param_t vz;
    pid_param_t x;
    pid_param_t y;
    pid_param_t z;
} pid_group_pos_t;

typedef struct
{
    float rateScaleRP;
    float rateScaleYaw;
    float angleScaleRP;
    float yawRateScale;
    float autolandDescent;
    float autolandRampStep;
    float takeoffRampStep;
    float takeoffMinThrust;
} commander_tune_t;

typedef struct
{
    uint8_t version;
    pid_group_t pidAngle;
    pid_group_t pidRate;
    pid_group_pos_t pidPos;
    float trimP;
    float trimR;
    uint16_t thrustBase;
    commander_tune_t cmdTune;
    uint8_t checksum;
} __attribute__((packed)) config_param_t;

#ifdef __cplusplus
}
#endif

#endif