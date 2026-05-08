// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  2nd-order Butterworth low-pass filter implementation.
 *
 * @details
 * Coefficients derived from the analog Butterworth prototype via the
 * bilinear (Tustin) transform. The damping factor alpha = sqrt(2) gives
 * a maximally flat passband response (Q = 1/sqrt(2) = 0.707).
 */

#include "platform/filter.h"

#include <math.h>

/* Portability guard -- some compilers don't define M_PI in <math.h> */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void lpf2p_init(lpf2pData *lpf, float sample_freq, float cutoff_freq)
{
  lpf2p_set_cutoff(lpf, sample_freq, cutoff_freq);
  lpf->delay_element_1 = 0.0f;
  lpf->delay_element_2 = 0.0f;
}

void lpf2p_set_cutoff(lpf2pData *lpf, float sample_freq, float cutoff_freq)
{
  float fr = sample_freq / cutoff_freq; /* frequency ratio */
  float omega = (float)tan(
    M_PI / fr); /* pre-warped analog frequency (bilinear transform) */
  float cs = 1.0f / omega;
  float alpha =
    1.41421356f; /* sqrt(2) -- Butterworth damping factor (Q = 0.707) */

  lpf->b0 = 1.0f / (1.0f + alpha * cs + cs * cs);
  lpf->b1 = 2.0f * lpf->b0;
  lpf->b2 = lpf->b0;
  lpf->a1 = 2.0f * lpf->b0 * (1.0f - cs * cs);
  lpf->a2 = lpf->b0 * (1.0f - alpha * cs + cs * cs);
}

float lpf2p_apply(lpf2pData *lpf, float sample)
{
  float delay_element_0 =
    sample - lpf->delay_element_1 * lpf->a1 - lpf->delay_element_2 * lpf->a2;

  /* Bypass NaN/Inf -- corrupted sensor data (e.g., I2C bus error)
     * would permanently poison all subsequent filter outputs. */
  if (isnan(delay_element_0) || isinf(delay_element_0)) {
    delay_element_0 = sample;
  }

  float output = delay_element_0 * lpf->b0 + lpf->delay_element_1 * lpf->b1 +
                 lpf->delay_element_2 * lpf->b2;

  lpf->delay_element_2 = lpf->delay_element_1;
  lpf->delay_element_1 = delay_element_0;

  return output;
}
