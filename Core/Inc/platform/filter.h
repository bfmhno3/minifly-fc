#ifndef PLATFORM_FILTER_H
#define PLATFORM_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float delay_element_1;
    float delay_element_2;
} lpf2pData;

void lpf2p_init(lpf2pData *lpf, float sample_freq, float cutoff_freq);
float lpf2p_apply(lpf2pData *lpf, float sample);
void lpf2p_set_cutoff(lpf2pData *lpf, float sample_freq, float cutoff_freq);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_FILTER_H */
