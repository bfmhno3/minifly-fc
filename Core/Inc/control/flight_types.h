// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Shared type definitions for the flight control pipeline
 *
 * @details
 * Defines the core data structures exchanged between estimator, PID,
 * and motor-mixing stages.  All angles are in degrees, all positions
 * in centimetres, and timestamps in milliseconds unless noted otherwise.
 */

#ifndef __FLIGHT_TYPES_H
#define __FLIGHT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Euler angle attitude in degrees (roll, pitch, yaw). */
typedef struct {
  uint32_t timestamp;
  float roll;
  float pitch;
  float yaw;
} attitude_t;

/** @brief Generic 3-component vector (position in cm, velocity in cm/s, or acceleration). */
typedef struct {
  uint32_t timestamp;
  float x;
  float y;
  float z;
} vec3_s;

typedef vec3_s point_t;
typedef vec3_s velocity_t;
typedef vec3_s acc_t;

/** @brief Unit quaternion for attitude representation (Hamilton convention). */
typedef struct {
  uint32_t timestamp;
  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

/** @brief Control mode for a single axis. */
typedef enum {
  MODE_DISABLE = 0,  /**< Axis not controlled */
  MODE_ABS = 1,      /**< Absolute position setpoint */
  MODE_VELOCITY = 2, /**< Velocity setpoint */
} mode_e;

typedef struct {
  mode_e x;
  mode_e y;
  mode_e z;
  mode_e roll;
  mode_e pitch;
  mode_e yaw;
} mode_t;

/** @brief Full vehicle state estimate (fused sensor output). */
typedef struct {
  attitude_t attitude;
  quaternion_t attitudeQuaternion;
  point_t position;    /**< cm, earth frame */
  velocity_t velocity; /**< cm/s, earth frame */
  acc_t acc;       /**< earth-frame acceleration (cm/s^2 or G, see estimator) */
  bool isRCLocked; /**< true when RC link is active */
} state_t;

/** @brief Commanded setpoint from the pilot or autopilot. */
typedef struct {
  attitude_t attitude;     /**< desired angle (deg) */
  attitude_t attitudeRate; /**< desired angular rate (deg/s), used in RATE mode */
  point_t position;        /**< desired position (cm) */
  velocity_t velocity;     /**< desired velocity (cm/s) */
  mode_t mode;             /**< per-axis control mode */
  float thrust;            /**< raw thrust command (0-65535) */
} setpoint_t;

/** @brief Flip direction (FLIP_DIR_CENTER means no flip active). */
typedef enum {
  FLIP_DIR_CENTER = 0,
  FLIP_DIR_FORWARD,
  FLIP_DIR_BACK,
  FLIP_DIR_LEFT,
  FLIP_DIR_RIGHT,
} flip_dir_e;

/** @brief Control output fed to the motor mixer. */
typedef struct {
  int16_t roll;        /**< roll effort (PID output, unitless) */
  int16_t pitch;       /**< pitch effort */
  int16_t yaw;         /**< yaw effort */
  float thrust;        /**< thrust command (0-65535 range) */
  flip_dir_e flip_dir; /**< active flip direction, FLIP_DIR_CENTER if idle */
} control_t;

#ifdef __cplusplus
}
#endif

#endif /* __FLIGHT_TYPES_H */
