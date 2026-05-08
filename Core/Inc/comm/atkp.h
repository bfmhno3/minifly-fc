// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  ATKP protocol definitions -- message IDs, frame format, and commands.
 *
 * @details
 * ATKP (AeroTelemetry Kontrol Protocol) is a binary framing protocol used for
 * bidirectional communication between the flight controller and a ground station.
 * Frames use XOR-based checksums and carry a single-byte message ID that
 * determines the payload layout.
 *
 * Up-link (drone -> ground) messages are periodic telemetry; down-link
 * (ground -> drone) messages carry commands, RC data, and PID tuning.
 */

#ifndef __ATKP_H
#define __ATKP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Start-of-frame magic byte. */
#define ATKP_START 0xAA

/** @brief Direction flag: drone-to-ground (up-link). */
#define ATKP_UP 0xAA

/** @brief Direction flag: ground-to-drone (down-link). */
#define ATKP_DOWN 0xAF

/** @brief Maximum payload bytes per frame. */
#define ATKP_FRAME_DATA_MAX 30

/**
 * @brief  Up-link message IDs (drone -> ground).
 *
 * Each ID maps to a fixed-size payload assembled by the periodic telemetry
 * sender in atkp.c.
 */
typedef enum {
  ATKP_UP_VERSION = 0x00,
  ATKP_UP_STATUS = 0x01,
  ATKP_UP_SENSOR = 0x02,
  ATKP_UP_RCDATA = 0x03,
  ATKP_UP_GPSDATA = 0x04,
  ATKP_UP_POWER = 0x05,
  ATKP_UP_MOTOR = 0x06,
  ATKP_UP_SENSOR2 = 0x07,
  ATKP_UP_FLYMODE = 0x0A,
  ATKP_UP_SPEED = 0x0B,
  ATKP_UP_PID1 = 0x10,
  ATKP_UP_PID2 = 0x11,
  ATKP_UP_PID3 = 0x12,
  ATKP_UP_PID4 = 0x13,
  ATKP_UP_PID5 = 0x14,
  ATKP_UP_PID6 = 0x15,
  ATKP_UP_RADIO = 0x40,
  ATKP_UP_REMOTER = 0x50,
  ATKP_UP_PRINTF = 0x51,
  ATKP_UP_MSG = 0xEE,
  ATKP_UP_CHECK = 0xEF,
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

/**
 * @brief  Down-link message IDs (ground -> drone).
 *
 * Carries commands, RC stick values, PID parameters, and radio config.
 */
typedef enum {
  ATKP_DOWN_COMMAND = 0x01,
  ATKP_DOWN_ACK = 0x02,
  ATKP_DOWN_RCDATA = 0x03,
  ATKP_DOWN_POWER = 0x05,
  ATKP_DOWN_FLYMODE = 0x0A,
  ATKP_DOWN_PID1 = 0x10,
  ATKP_DOWN_PID2 = 0x11,
  ATKP_DOWN_PID3 = 0x12,
  ATKP_DOWN_PID4 = 0x13,
  ATKP_DOWN_PID5 = 0x14,
  ATKP_DOWN_PID6 = 0x15,
  ATKP_DOWN_RADIO = 0x40,
  ATKP_DOWN_REMOTER = 0x50,
} atkp_down_msg_id_t;

/**
 * @name Command sub-IDs (payload of ATKP_DOWN_COMMAND)
 * @{
 */
#define ATKP_CMD_ACC_CALIB 0x01
#define ATKP_CMD_GYRO_CALIB 0x02
#define ATKP_CMD_MAG_CALIB 0x04
#define ATKP_CMD_BARO_CALIB 0x05
#define ATKP_CMD_ACC_CALIB_EXIT 0x20
#define ATKP_CMD_ACC_CALIB_STEP1 0x21
#define ATKP_CMD_ACC_CALIB_STEP2 0x22
#define ATKP_CMD_ACC_CALIB_STEP3 0x23
#define ATKP_CMD_ACC_CALIB_STEP4 0x24
#define ATKP_CMD_ACC_CALIB_STEP5 0x25
#define ATKP_CMD_ACC_CALIB_STEP6 0x26
#define ATKP_CMD_FLIGHT_LOCK 0xA0
#define ATKP_CMD_FLIGHT_ULOCK 0xA1
/** @} */

/**
 * @name ACK sub-IDs (payload of ATKP_DOWN_ACK)
 * @{
 */
#define ATKP_ACK_READ_PID 0x01
#define ATKP_ACK_READ_VERSION 0xA0
#define ATKP_ACK_RESET_PARAM 0xA1
/** @} */

/**
 * @brief  Decoded ATKP frame (header + payload).
 *
 * Passed between the transport layer and the application layer.
 * @c data_len bytes of @c data are valid when received from the queue.
 */
typedef struct {
  uint8_t msg_id;
  uint8_t data_len;
  uint8_t data[ATKP_FRAME_DATA_MAX];
} atkp_frame_t;

/**
 * @brief  Initialize the ATKP aggregation layer.
 *
 * Creates the internal RX queue.  Must be called before any other atkp function.
 */
void atkp_init(void);

/**
 * @brief  FreeRTOS task -- periodic telemetry sender.
 *
 * Sends up-link frames at fixed intervals over both radio and USB links.
 * Runs indefinitely; @c arg is unused.
 *
 * @param[in] arg  Unused.
 */
void atkp_tx_task(void *arg);

/**
 * @brief  FreeRTOS task -- dual-link frame receiver.
 *
 * Polls radiolink then usblink for received frames and enqueues them
 * into the internal RX queue.  Radio has priority over USB.
 * Runs indefinitely; @c arg is unused.
 *
 * @param[in] arg  Unused.
 */
void atkp_rx_task(void *arg);

/**
 * @brief  Dequeue one received ATKP frame (non-blocking).
 *
 * @param[out] frame  Destination for the dequeued frame.
 *
 * @retval true   A frame was dequeued.
 * @retval false  The RX queue was empty.
 */
bool atkp_receive_packet(atkp_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __ATKP_H */
