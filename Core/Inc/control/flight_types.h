#ifndef __FLIGHT_TYPES_H
#define __FLIGHT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t timestamp;
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct {
    uint32_t timestamp;
    float x;
    float y;
    float z;
} vec3_s;

typedef vec3_s point_t;
typedef vec3_s velocity_t;
typedef vec3_s acc_t;

typedef struct {
    uint32_t timestamp;
    union {
        struct { float q0; float q1; float q2; float q3; };
        struct { float x; float y; float z; float w; };
    };
} quaternion_t;

typedef enum {
    MODE_DISABLE  = 0,
    MODE_ABS      = 1,
    MODE_VELOCITY = 2,
} mode_e;

typedef struct {
    mode_e x; mode_e y; mode_e z;
    mode_e roll; mode_e pitch; mode_e yaw;
} mode_t;

typedef struct {
    attitude_t attitude;
    quaternion_t attitudeQuaternion;
    point_t position;
    velocity_t velocity;
    acc_t acc;
    bool isRCLocked;
} state_t;

typedef struct {
    attitude_t attitude;
    attitude_t attitudeRate;
    point_t position;
    velocity_t velocity;
    mode_t mode;
    float thrust;
} setpoint_t;

#ifdef __cplusplus
}
#endif

#endif /* __FLIGHT_TYPES_H */
