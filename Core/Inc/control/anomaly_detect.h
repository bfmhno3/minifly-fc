// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  In-flight anomaly detection (free-fall and tumble)
 *
 * @details
 * Monitors accelerometer magnitude and attitude angles to detect two
 * failure modes: free-fall (near-zero acceleration) and tumble (excessive
 * roll/pitch).  Returns true when motors should be cut for safety.
 */

#ifndef CONTROL_ANOMALY_DETECT_H
#define CONTROL_ANOMALY_DETECT_H

#include <stdbool.h>
#include "control/flight_types.h"
#include "services/sensors_types.h"

void anomaly_detect_init(void);

/**
 * @brief  Check for free-fall or tumble anomaly
 *
 * Returns false during active flips (cooldown period).
 * When an anomaly is detected, the caller should cut motors immediately.
 *
 * @param[in] sensor  Raw accelerometer data (body frame, G)
 * @param[in] state   Fused attitude estimate
 * @param[in] ctl     Current control output (used to check flip state)
 *
 * @return true if an anomaly is detected (motors should be cut)
 */
bool anomaly_detect_check(const sensor_data_t *sensor,
			   const state_t *state,
			   const control_t *ctl);

#endif /* CONTROL_ANOMALY_DETECT_H */
