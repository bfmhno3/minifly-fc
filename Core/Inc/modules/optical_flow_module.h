// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  PMW3901 optical flow sensor expansion module.
 *
 * @details
 * Reads pixel motion data from a PMW3901 optical flow sensor over SPI and
 * converts it to velocity and position estimates in the body frame.  The
 * sensor is mounted on the expansion module connector and managed by
 * module_manager.
 *
 * Hardware dependencies:
 * - SPI2 for PMW3901 communication.
 * - NCS (chip select) on PA8, shared with MODULE_SHARED_SCL.
 * - Module power pin via bsp_module BSP.
 */

#ifndef MODULES_OPTICAL_FLOW_MODULE_H
#define MODULES_OPTICAL_FLOW_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "control/flight_types.h"

/**
 * @brief  Optical flow data snapshot returned by optical_flow_module_get_data().
 */
typedef struct {
    bool valid;              /**< data is valid and usable */
    float pos_sum[2];        /**< accumulated position, cm [x, y] */
    float vel_lpf[2];        /**< filtered velocity, cm/s [x, y] */
    float laser_range;       /**< laser rangefinder distance, cm (unused if no rangefinder) */
    float laser_quality;     /**< laser rangefinder quality [0.0, 1.0] */
} optical_flow_data_t;

/**
 * @brief  Initialize the optical flow module.
 *
 * Configures the NCS GPIO, powers on the sensor, performs PMW3901
 * register initialization, and starts the FreeRTOS sampling task.
 * Safe to call multiple times (re-init resumes the task).
 */
void optical_flow_module_init(void);

/**
 * @brief  De-initialize the optical flow module.
 *
 * Suspends the sampling task and cuts power to the sensor.
 */
void optical_flow_module_deinit(void);

/**
 * @brief  Update position and velocity estimates from optical flow data.
 *
 * Converts raw pixel deltas to centimetres using the current height estimate
 * and attitude, applies a low-pass filter to velocity, and accumulates
 * position.  Should be called from the stabilizer loop at a fixed rate.
 *
 * @param[in] state  Current attitude state (for compensation).
 * @param[in] dt     Time step in seconds since last call.
 */
void optical_flow_module_update(state_t *state, float dt);

/**
 * @brief  Check whether the optical flow data is currently valid.
 *
 * Data becomes invalid if the sensor fails or the height exceeds the
 * operating range.
 *
 * @return true if data is valid.
 */
bool optical_flow_module_is_valid(void);

/**
 * @brief  Copy the latest optical flow data snapshot.
 *
 * @param[out] out  Pointer to receive the data.  Always written (check
 *                  out->valid before using values).
 */
void optical_flow_module_get_data(optical_flow_data_t *out);

#endif /* MODULES_OPTICAL_FLOW_MODULE_H */
