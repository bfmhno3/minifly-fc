#ifndef __SENSFUSION6_H
#define __SENSFUSION6_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * 6�������ںϴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

void imuUpdate(axis3f_t acc, axis3f_t gyro, state_t *state , float dt);	/*�����ں� �����˲�*/
bool getIsCalibrated(void);
void imuTransformVectorBodyToEarth(axis3f_t * v);	/*���嵽����*/
void imuTransformVectorEarthToBody(axis3f_t * v);	/*���򵽻���*/

#endif

