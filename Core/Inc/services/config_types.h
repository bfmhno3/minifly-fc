// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Configuration parameter type definitions.
 *
 * @details
 * Defines the persistent configuration structure (config_param_t) and its
 * sub-types for PID gains, trim values, and commander tuning parameters.
 * The struct layout is packed for direct flash serialization.
 */

#ifndef CONFIG_TYPES_H
#define CONFIG_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief  Flash storage schema version. Increment when struct layout changes. */
#define CONFIG_VERSION 13

/**
 * @brief  Single-axis PID gains.
 */
typedef struct {
  float kp; /**< Proportional gain */
  float ki; /**< Integral gain */
  float kd; /**< Derivative gain */
} pid_param_t;

/**
 * @brief  Three-axis PID group (roll, pitch, yaw) for angle or rate loops.
 */
typedef struct {
  pid_param_t roll;
  pid_param_t pitch;
  pid_param_t yaw;
} pid_group_t;

/**
 * @brief  Six-axis PID group for position/velocity loops (vx, vy, vz, x, y, z).
 */
typedef struct {
  pid_param_t vx;
  pid_param_t vy;
  pid_param_t vz;
  pid_param_t x;
  pid_param_t y;
  pid_param_t z;
} pid_group_pos_t;

/**
 * @brief  Commander input scaling and autonomous-flight tuning parameters.
 */
typedef struct {
  float rateScaleRP; /**< Max angular rate (deg/s) for roll/pitch stick input */
  float rateScaleYaw;     /**< Max angular rate (deg/s) for yaw stick input */
  float angleScaleRP;     /**< Max angle (deg) for roll/pitch stick input */
  float yawRateScale;     /**< Yaw rate scaling factor */
  float autolandDescent;  /**< Autoland descent rate (m/s) */
  float autolandRampStep; /**< Autoland thrust ramp step per tick */
  float takeoffRampStep;  /**< Takeoff thrust ramp step per tick */
  float takeoffMinThrust; /**< Minimum thrust during takeoff ramp */
} commander_tune_t;

/**
 * @brief  Top-level persistent configuration stored in internal flash.
 *
 * @note  Packed layout -- used directly for flash read/write.
 *        Must bump CONFIG_VERSION when fields change.
 */
typedef struct {
  uint8_t version;
  pid_group_t pidAngle;     /**< Angle-loop PID gains */
  pid_group_t pidRate;      /**< Rate-loop PID gains */
  pid_group_pos_t pidPos;   /**< Position/velocity-loop PID gains */
  float trimP;              /**< Pitch trim (degrees) */
  float trimR;              /**< Roll trim (degrees) */
  uint16_t thrustBase;      /**< Hover thrust baseline (PWM units) */
  commander_tune_t cmdTune; /**< Commander input scaling */
  uint8_t checksum;         /**< Simple byte-sum integrity check */
} __attribute__((packed)) config_param_t;

#ifdef __cplusplus
}
#endif

#endif