// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  USB CDC transport layer.
 *
 * @details
 * Provides a frame-based bidirectional link over USB CDC (Virtual COM Port).
 * RX bytes arrive via the CDC receive callback (ISR context) into a shared
 * byte queue; the RX task parses them into ATKP frames.  TX frames are
 * queued and sent by the TX task through the CDC bulk-in endpoint.
 *
 * The rx_queue and tx_done handles are exposed as globals so that the
 * CDC ISR bridge in usbd_cdc_if.c can push bytes directly without
 * coupling to this module's internals.
 */

#ifndef __USBLINK_H
#define __USBLINK_H

#include <stdbool.h>
#include "comm/atkp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the USB link.
 *
 * Creates RX/TX queues and the TX-done semaphore.
 * Safe to call multiple times (subsequent calls are no-ops).
 */
void usblink_init(void);

/**
 * @brief  FreeRTOS task -- USB RX frame parser.
 *
 * Dequeues bytes from the shared rx_queue (filled by the CDC ISR)
 * and feeds them into the ATKP frame state machine.
 * Runs indefinitely; @c arg is unused.
 *
 * @param[in] arg  Unused.
 */
void usblink_rx_task(void *arg);

/**
 * @brief  FreeRTOS task -- USB frame transmitter.
 *
 * Dequeues frames from the TX queue, assembles ATKP packets, and sends
 * them via CDC_Transmit_FS.  Retries on USBD_BUSY.
 * Runs indefinitely; @c arg is unused.
 *
 * @param[in] arg  Unused.
 */
void usblink_tx_task(void *arg);

/**
 * @brief  Dequeue one received USB frame (non-blocking).
 *
 * @param[out] frame  Destination for the dequeued frame.
 *
 * @retval true   A complete frame was available.
 * @retval false  No frame ready since last call.
 */
bool usblink_get_frame(atkp_frame_t *frame);

/**
 * @brief  Enqueue a frame for USB transmission (non-blocking).
 *
 * @param[in] frame  Frame to enqueue.
 *
 * @retval true   Frame was enqueued.
 * @retval false  TX queue is full -- frame dropped.
 */
bool usblink_send_frame(const atkp_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __USBLINK_H */
