#ifndef __ATKP_H
#define __ATKP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATKP_START 0xAA
#define ATKP_UP    0xAA
#define ATKP_DOWN  0xAF

#define ATKP_FRAME_DATA_MAX 30

typedef enum {
	ATKP_UP_VERSION    = 0x00,
	ATKP_UP_STATUS     = 0x01,
	ATKP_UP_SENSOR     = 0x02,
	ATKP_UP_RCDATA     = 0x03,
	ATKP_UP_GPSDATA    = 0x04,
	ATKP_UP_POWER      = 0x05,
	ATKP_UP_MOTOR      = 0x06,
	ATKP_UP_SENSOR2    = 0x07,
	ATKP_UP_FLYMODE    = 0x0A,
	ATKP_UP_SPEED      = 0x0B,
	ATKP_UP_PID1       = 0x10,
	ATKP_UP_PID2       = 0x11,
	ATKP_UP_PID3       = 0x12,
	ATKP_UP_PID4       = 0x13,
	ATKP_UP_PID5       = 0x14,
	ATKP_UP_PID6       = 0x15,
	ATKP_UP_RADIO      = 0x40,
	ATKP_UP_REMOTER    = 0x50,
	ATKP_UP_PRINTF     = 0x51,
	ATKP_UP_MSG        = 0xEE,
	ATKP_UP_CHECK      = 0xEF,
	ATKP_UP_USER_DATA1 = 0xF1,
	ATKP_UP_USER_DATA2 = 0xF2,
	ATKP_UP_USER_DATA3 = 0xF3,
	ATKP_UP_USER_DATA4 = 0xF4,
	ATKP_UP_USER_DATA5 = 0xF5,
	ATKP_UP_USER_DATA6 = 0xF6,
	ATKP_UP_USER_DATA7 = 0xF7,
	ATKP_UP_USER_DATA8 = 0xF8,
	ATKP_UP_USER_DATA9 = 0xF9,
	ATKP_UP_USER_DATA10 = 0xFA,
} atkp_up_msg_id_t;

typedef enum {
	ATKP_DOWN_COMMAND = 0x01,
	ATKP_DOWN_ACK     = 0x02,
	ATKP_DOWN_RCDATA  = 0x03,
	ATKP_DOWN_POWER   = 0x05,
	ATKP_DOWN_FLYMODE = 0x0A,
	ATKP_DOWN_PID1    = 0x10,
	ATKP_DOWN_PID2    = 0x11,
	ATKP_DOWN_PID3    = 0x12,
	ATKP_DOWN_PID4    = 0x13,
	ATKP_DOWN_PID5    = 0x14,
	ATKP_DOWN_PID6    = 0x15,
	ATKP_DOWN_RADIO   = 0x40,
	ATKP_DOWN_REMOTER = 0x50,
} atkp_down_msg_id_t;

#define ATKP_CMD_ACC_CALIB       0x01
#define ATKP_CMD_GYRO_CALIB      0x02
#define ATKP_CMD_MAG_CALIB       0x04
#define ATKP_CMD_BARO_CALIB      0x05
#define ATKP_CMD_ACC_CALIB_EXIT  0x20
#define ATKP_CMD_ACC_CALIB_STEP1 0x21
#define ATKP_CMD_ACC_CALIB_STEP2 0x22
#define ATKP_CMD_ACC_CALIB_STEP3 0x23
#define ATKP_CMD_ACC_CALIB_STEP4 0x24
#define ATKP_CMD_ACC_CALIB_STEP5 0x25
#define ATKP_CMD_ACC_CALIB_STEP6 0x26
#define ATKP_CMD_FLIGHT_LOCK     0xA0
#define ATKP_CMD_FLIGHT_ULOCK    0xA1

#define ATKP_ACK_READ_PID     0x01
#define ATKP_ACK_READ_VERSION 0xA0
#define ATKP_ACK_RESET_PARAM  0xA1

typedef struct {
	uint8_t msg_id;
	uint8_t data_len;
	uint8_t data[ATKP_FRAME_DATA_MAX];
} atkp_frame_t;

void atkp_init(void);
void atkp_tx_task(void *arg);
void atkp_rx_task(void *arg);
bool atkp_receive_packet(atkp_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __ATKP_H */
