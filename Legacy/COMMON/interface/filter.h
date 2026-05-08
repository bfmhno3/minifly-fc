#ifndef __FILTER_H
#define __FILTER_H
#include <stdint.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �˲����ܺ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define IIR_SHIFT         8

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);

typedef struct 
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;
	float delay_element_2;
} lpf2p_data;

void lpf2pInit(lpf2p_data* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2p_data* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2p_data* lpfData, float sample);
float lpf2pReset(lpf2p_data* lpfData, float sample);


#endif //__FILTER_H
