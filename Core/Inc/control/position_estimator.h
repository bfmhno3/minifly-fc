// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Position and velocity estimator (INAV-style complementary filter)
 *
 * @details
 * Fuses barometer, accelerometer, and optional optical-flow/laser-ranger
 * data to produce 3D position and velocity estimates in the earth frame.
 * Height is corrected by barometer; XY is corrected by optical flow when
 * available.
 */

#ifndef CONTROL_POSITION_ESTIMATOR_H
#define CONTROL_POSITION_ESTIMATOR_H

#include <stdbool.h>

#include "control/flight_types.h"
#include "services/sensors_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief External sensor data from the optical flow module */
typedef struct {
  bool valid;          /**< true when optical flow data is usable */
  float pos_sum[2];    /**< integrated XY position (cm) */
  float vel_lpf[2];    /**< filtered XY velocity (cm/s) */
  float laser_range;   /**< laser ranger distance (cm) */
  float laser_quality; /**< laser signal quality [0..1] */
} pos_estimator_ext_t;

void position_estimator_init(void);

/**
 * @brief  Run one position-estimator update cycle
 *
 * @param[out] state  Fused state to write (acc, velocity, position)
 * @param[in]  sensor Raw sensor readings (baro, acc)
 * @param[in]  ext    Optical flow / laser data (may be NULL)
 * @param[in]  dt     Time step in seconds
 */
void position_estimator_update(state_t *state, const sensor_data_t *sensor,
                               const pos_estimator_ext_t *ext, float dt);

/** @brief  Request a height-only reset (re-seed from baro + laser) */
void estimator_reset_height(void);

/** @brief  Request a full state reset (zero velocity, re-seed position) */
void estimator_reset_all(void);

/** @brief  Get the low-pass-filtered fused height (cm) */
float position_estimator_get_fused_height(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_POSITION_ESTIMATOR_H */
