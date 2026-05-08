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
typedef struct attitude {
  uint32_t timestamp;
  float roll;
  float pitch;
  float yaw;
} attitude_t;

/** @brief Generic 3-component vector (position in cm, velocity in cm/s, or acceleration). */
typedef struct vec3 {
  uint32_t timestamp;
  float x;
  float y;
  float z;
} vec3_t;

typedef vec3_t point_t;
typedef vec3_t velocity_t;
typedef vec3_t acc_t;

/** @brief Unit quaternion for attitude representation (Hamilton convention). */
typedef struct quaternion {
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
typedef enum axis_mode {
  MODE_DISABLE = 0,  /**< Axis not controlled */
  MODE_ABS = 1,      /**< Absolute position setpoint */
  MODE_VELOCITY = 2, /**< Velocity setpoint */
} axis_mode_t;

/** @brief Per-axis control mode selection for the commanded setpoint. */
typedef struct setpoint_mode {
  axis_mode_t x;
  axis_mode_t y;
  axis_mode_t z;
  axis_mode_t roll;
  axis_mode_t pitch;
  axis_mode_t yaw;
} setpoint_mode_t;

/** @brief Full vehicle state estimate (fused sensor output). */
typedef struct state {
  attitude_t attitude;
  quaternion_t attitude_quaternion;
  point_t position;    /**< cm, earth frame */
  velocity_t velocity; /**< cm/s, earth frame */
  acc_t acc;       /**< earth-frame acceleration (cm/s^2 or G, see estimator) */
  bool is_rc_locked; /**< true when RC link is active */
} state_t;

/** @brief Commanded setpoint from the pilot or autopilot. */
typedef struct setpoint {
  attitude_t attitude;     /**< desired angle (deg) */
  attitude_t attitude_rate; /**< desired angular rate (deg/s), used in RATE mode */
  point_t position;        /**< desired position (cm) */
  velocity_t velocity;     /**< desired velocity (cm/s) */
  setpoint_mode_t mode;    /**< per-axis control mode */
  float thrust;            /**< raw thrust command (0-65535) */
} setpoint_t;

/** @brief Flip direction (FLIP_DIR_CENTER means no flip active). */
typedef enum flip_dir {
  FLIP_DIR_CENTER = 0,
  FLIP_DIR_FORWARD,
  FLIP_DIR_BACK,
  FLIP_DIR_LEFT,
  FLIP_DIR_RIGHT,
} flip_dir_e;

/** @brief Control output fed to the motor mixer. */
typedef struct control {
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
