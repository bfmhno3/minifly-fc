#ifndef PLATFORM_AXIS_H
#define PLATFORM_AXIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

typedef union {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
} Axis3i16;

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_AXIS_H */
