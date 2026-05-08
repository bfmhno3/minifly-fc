#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���������ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

//#define SENSORS_ENABLE_MAG_AK8963
#define SENSORS_ENABLE_PRESSURE_BMP280	/*��ѹ��ʹ��bmp280*/

#define BARO_UPDATE_RATE		RATE_50_HZ
#define SENSOR9_UPDATE_RATE   	RATE_500_HZ
#define SENSOR9_UPDATE_DT     	(1.0f/SENSOR9_UPDATE_RATE)

	
void sensorsTask(void *param);
void sensorsInit(void);			/*��������ʼ��*/
bool sensorsTest(void);			/*����������*/
bool sensorsAreCalibrated(void);	/*����������У׼*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick);/*��ȡ����������*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag);
bool getIsMPU9250Present(void);
bool getIsBaroPresent(void);

/* ������������������ */
bool sensorsReadGyro(axis3f_t *gyro);
bool sensorsReadAcc(axis3f_t *acc);
bool sensorsReadMag(axis3f_t *mag);
bool sensorsReadBaro(baro_t *baro);

#endif //__SENSORS_H
