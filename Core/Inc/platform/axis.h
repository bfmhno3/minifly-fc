// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Canonical 3-axis type definitions.
 *
 * @details
 * Provides union-based axis types used across sensors, control, and math
 * layers. The union layout allows both named-field access (x, y, z) and
 * array-based iteration (axis[3]) with guaranteed layout compatibility.
 */

#ifndef PLATFORM_AXIS_H
#define PLATFORM_AXIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief  Axis indices for array-based access to 3-axis data. */
enum {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z
};
#define AXIS_COUNT 3 /** Number of spatial axes */

/**
 * @brief  3-axis float vector with dual access pattern.
 *
 * Named fields (x, y, z) for readability; array view (axis[3]) for
 * loop-based iteration. Union guarantees layout compatibility.
 */
typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

/**
 * @brief  3-axis int16 vector (sensor raw data variant).
 *
 * Same dual-access pattern as Axis3f.
 */
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
