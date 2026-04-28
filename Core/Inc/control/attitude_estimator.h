#ifndef CONTROL_ATTITUDE_ESTIMATOR_H
#define CONTROL_ATTITUDE_ESTIMATOR_H

#include "platform/axis.h"
#include "control/flight_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void attitude_estimator_init(void);

/**
 * @brief Fuse gyro and accelerometer data to update attitude estimate.
 * @param acc  Accelerometer reading, body frame (normalized internally)
 * @param gyro Gyroscope reading, body frame (deg/s)
 * @param dt   Time step in seconds (typically 0.004 for 250 Hz)
 */
void attitude_estimator_update(const Axis3f *acc, const Axis3f *gyro, float dt);

/**
 * @brief Get the latest attitude estimate.
 * @param[out] attitude Roll/pitch/yaw in degrees
 */
void attitude_estimator_get_attitude(attitude_t *attitude);

/**
 * @brief Get the current attitude quaternion.
 * @param[out] quat Output quaternion
 */
void attitude_estimator_get_quaternion(quaternion_t *quat);

/**
 * @brief Check if gravity calibration has completed.
 * @return true if calibrated
 */
bool attitude_estimator_is_calibrated(void);

/**
 * @brief Transform a vector from body frame to earth frame.
 * @param[in,out] v Overwritten with earth-frame result
 */
void attitude_estimator_body_to_earth(Axis3f *v);

/**
 * @brief Transform a vector from earth frame to body frame.
 * @param[in,out] v Overwritten with body-frame result
 */
void attitude_estimator_earth_to_body(Axis3f *v);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_ATTITUDE_ESTIMATOR_H */
