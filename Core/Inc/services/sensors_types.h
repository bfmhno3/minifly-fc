#ifndef SERVICES_SENSORS_TYPES_H
#define SERVICES_SENSORS_TYPES_H

#include "platform/axis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pressure;    /* hPa */
    float temperature; /* deg C */
    float asl;         /* altitude above sea level, cm */
} baro_t;

typedef struct {
    Axis3f acc;
    Axis3f gyro;
    Axis3f mag;
    baro_t baro;
} sensor_data_t;

#define RATE_5_HZ    5
#define RATE_10_HZ   10
#define RATE_25_HZ   25
#define RATE_50_HZ   50
#define RATE_100_HZ  100
#define RATE_200_HZ  200
#define RATE_250_HZ  250
#define RATE_500_HZ  500
#define RATE_1000_HZ 1000

#define MAIN_LOOP_RATE     RATE_1000_HZ
#define MAIN_LOOP_DT       (1000U / MAIN_LOOP_RATE)

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK) % (MAIN_LOOP_RATE / (RATE_HZ)) == 0U)

#define BARO_UPDATE_RATE    RATE_50_HZ
#define SENSOR9_UPDATE_RATE RATE_500_HZ
#define SENSOR9_UPDATE_DT   (1.0f / SENSOR9_UPDATE_RATE)

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_SENSORS_TYPES_H */
