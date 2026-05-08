// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  2nd-order IIR low-pass filter (Butterworth).
 *
 * @details
 * Direct Form II Transposed structure with runtime-adjustable cutoff.
 * Coefficients are recomputed via bilinear (Tustin) transform when the
 * cutoff frequency changes. No hardware dependencies.
 */

#ifndef PLATFORM_FILTER_H
#define PLATFORM_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  2nd-order IIR filter state (Direct Form II Transposed).
 *
 * lpf: low-pass filter
 * 2p: 2-pole/2nd-order
 *
 * b0/b1/b2 = numerator coefficients, a1/a2 = denominator coefficients.
 * delay_element_1/2 = filter state (z^-1, z^-2).
 */
typedef struct lpf2p_data {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float delay_element_1;
  float delay_element_2;
} lpf2p_data_t;

/**
 * @brief  Initialize filter with given sample and cutoff frequencies.
 *
 * @param[out] lpf          Filter state to initialize.
 * @param[in]  sample_freq  Sampling frequency in Hz.
 * @param[in]  cutoff_freq  Cutoff frequency in Hz.
 */
void lpf2p_init(lpf2p_data_t *lpf, float sample_freq, float cutoff_freq);

/**
 * @brief  Apply filter to one sample.
 *
 * @param[in,out] lpf     Filter state.
 * @param[in]     sample  Input sample.
 * @return Filtered output.
 *
 * @note NaN/Inf inputs are bypassed to prevent filter state corruption
 *       (e.g., from I2C bus errors injecting garbage sensor data).
 */
float lpf2p_apply(lpf2p_data_t *lpf, float sample);

/**
 * @brief  Recompute filter coefficients for a new cutoff frequency.
 *
 * Uses bilinear (Tustin) transform. Does not reset filter state --
 * the transition is smooth.
 *
 * @param[out] lpf          Filter state to update.
 * @param[in]  sample_freq  Sampling frequency in Hz.
 * @param[in]  cutoff_freq  New cutoff frequency in Hz.
 */
void lpf2p_set_cutoff(lpf2p_data_t *lpf, float sample_freq, float cutoff_freq);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_FILTER_H */
