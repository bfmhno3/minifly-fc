#include "platform/filter.h"
#include <math.h>

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
    float fr = sample_freq / cutoff_freq;
    float omega = (float)tan(M_PI / fr);
    float cs = 1.0f / omega;
    float alpha = 1.41421356f;

    lpf->b0 = 1.0f / (1.0f + alpha * cs + cs * cs);
    lpf->b1 = 2.0f * lpf->b0;
    lpf->b2 = lpf->b0;
    lpf->a1 = 2.0f * lpf->b0 * (1.0f - cs * cs);
    lpf->a2 = lpf->b0 * (1.0f - alpha * cs + cs * cs);
}

float lpf2p_apply(lpf2pData *lpf, float sample)
{
    float delay_element_0 = sample - lpf->delay_element_1 * lpf->a1 - lpf->delay_element_2 * lpf->a2;

    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        delay_element_0 = sample;
    }

    float output = delay_element_0 * lpf->b0 + lpf->delay_element_1 * lpf->b1 + lpf->delay_element_2 * lpf->b2;

    lpf->delay_element_2 = lpf->delay_element_1;
    lpf->delay_element_1 = delay_element_0;

    return output;
}
