// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Radio transport layer -- nRF24L01 over USART2 with DMA TX.
 *
 * @details
 * Provides a frame-based bidirectional link over the nRF24L01 radio module
 * connected via USART2.  Reception is interrupt-driven (byte-by-byte via
 * HAL UART RX IT); transmission uses DMA.  An EXTI0 interrupt on PA0
 * handles the nRF24L01 flow-control signal.
 *
 * All public API functions are safe to call from task context only
 * (not from ISR).
 */

#ifndef __RADIOLINK_H
#define __RADIOLINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum payload bytes per radio frame. */
#define RADIOLINK_FRAME_DATA_MAX 30

/**
 * @brief  Radio transport frame (header + payload).
 *
 * Layout mirrors atkp_frame_t so the two can be cast-compatible,
 * but uses its own type to keep the transport layer decoupled.
 */
typedef struct radio_frame {
  uint8_t msg_id;
  uint8_t data_len;
  uint8_t data[RADIOLINK_FRAME_DATA_MAX];
} radio_frame_t;

/**
 * @brief  Initialize the radio link.
 *
 * Creates RX/TX queues and the TX-done semaphore.
 * Enables EXTI0 on PA0 for nRF24L01 flow control and starts
 * the first UART RX interrupt.  Safe to call multiple times
 * (subsequent calls are no-ops).
 */
void radiolink_init(void);

/**
 * @brief  FreeRTOS task -- radio link main loop.
 *
 * Blocks on the RX queue with a timeout.  When a byte arrives,
 * feeds it into the frame parser; on timeout, attempts to send
 * the next queued TX frame via DMA.  Runs indefinitely.
 *
 * @param[in] arg  Unused.
 */
void radiolink_task(void *arg);

/**
 * @brief  Dequeue one received radio frame (non-blocking).
 *
 * @param[out] frame  Destination for the dequeued frame.
 *
 * @retval true   A complete frame was available.
 * @retval false  No frame ready since last call.
 */
bool radiolink_get_frame(radio_frame_t *frame);

/**
 * @brief  Enqueue a frame for transmission (non-blocking).
 *
 * The frame will be sent by radiolink_task when the bus is idle.
 *
 * @param[in] frame  Frame to enqueue.
 *
 * @retval true   Frame was enqueued.
 * @retval false  TX queue is full -- frame dropped.
 */
bool radiolink_send_frame(const radio_frame_t *frame);

/** @brief  EXTI callback for nRF24L01 flow control (PA0). Called from platform_irq dispatcher. */
void radiolink_exti_callback(uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __RADIOLINK_H */
