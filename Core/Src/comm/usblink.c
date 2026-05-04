// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  USB CDC transport implementation.
 *
 * @details
 * RX path: CDC receive callback (ISR) pushes bytes into usblink_rx_queue ->
 *          usblink_rx_task dequeues and feeds process_byte() -> completed
 *          frame stored for usblink_get_frame().
 *
 * TX path: usblink_send_frame() enqueues into tx_queue -> usblink_tx_task
 *          assembles ATKP packet and calls CDC_Transmit_FS -> waits on
 *          usblink_tx_done semaphore signaled by the CDC transfer-complete
 *          callback.
 */

#include "comm/usblink.h"

#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "queue.h"

/** @brief RX byte queue depth. */
#define USBLINK_RX_QUEUE_BYTES 1024

/** @brief TX frame queue depth. */
#define USBLINK_TX_QUEUE_FRAMES 30

/** @brief Retry delay when CDC_Transmit_FS returns USBD_BUSY (ms). */
#define USBLINK_TX_RETRY_MS 1

/** @brief RX frame parser states. */
enum rx_state {
	STATE_START1,
	STATE_START2,
	STATE_MSG_ID,
	STATE_DATA_LEN,
	STATE_DATA,
	STATE_CHKSUM,
};

/*
 * Handles exposed for the CDC ISR bridge in usbd_cdc_if.c.
 * The CDC receive callback pushes raw bytes into usblink_rx_queue;
 * the transfer-complete callback gives usblink_tx_done.
 */
QueueHandle_t usblink_rx_queue;
SemaphoreHandle_t usblink_tx_done;

static struct {
	QueueHandle_t tx_queue;

	enum rx_state state;
	atkp_frame_t rx_frame;
	uint8_t data_index;
	uint8_t checksum;

	atkp_frame_t output;
	bool has_frame;

	bool is_init;
} g_usblink;

/**
 * @brief  Feed one byte into the RX state machine.
 *
 * Accepts only ATKP_DOWN direction (ground -> drone).
 * On checksum match the frame is copied to the output buffer.
 */
static void process_byte(uint8_t byte)
{
	switch (g_usblink.state) {
	case STATE_START1:
		if (byte == ATKP_START)
			g_usblink.state = STATE_START2;
		break;

	case STATE_START2:
		if (byte == ATKP_DOWN) {
			g_usblink.state = STATE_MSG_ID;
		} else if (byte != ATKP_START) {
			g_usblink.state = STATE_START1;
		}
		break;

	case STATE_MSG_ID:
		g_usblink.rx_frame.msg_id = byte;
		g_usblink.checksum = byte;
		g_usblink.state = STATE_DATA_LEN;
		break;

	case STATE_DATA_LEN:
		g_usblink.rx_frame.data_len = byte;
		g_usblink.checksum ^= byte;

		if (byte > ATKP_FRAME_DATA_MAX) {
			g_usblink.state = STATE_START1;
		} else if (byte == 0) {
			g_usblink.state = STATE_CHKSUM;
		} else {
			g_usblink.data_index = 0;
			g_usblink.state = STATE_DATA;
		}
		break;

	case STATE_DATA:
		g_usblink.rx_frame.data[g_usblink.data_index] = byte;
		g_usblink.checksum ^= byte;
		g_usblink.data_index++;

		if (g_usblink.data_index >= g_usblink.rx_frame.data_len)
			g_usblink.state = STATE_CHKSUM;
		break;

	case STATE_CHKSUM:
		if (byte == g_usblink.checksum) {
			g_usblink.output = g_usblink.rx_frame;
			g_usblink.has_frame = true;
		}
		g_usblink.state = STATE_START1;
		break;
	}
}

void usblink_init(void)
{
	if (g_usblink.is_init)
		return;

	usblink_rx_queue = xQueueCreate(USBLINK_RX_QUEUE_BYTES, sizeof(uint8_t));
	g_usblink.tx_queue = xQueueCreate(USBLINK_TX_QUEUE_FRAMES, sizeof(atkp_frame_t));
	usblink_tx_done = xSemaphoreCreateBinary();

	g_usblink.state = STATE_START1;
	g_usblink.has_frame = false;
	g_usblink.is_init = true;
}

void usblink_rx_task(void *arg)
{
	uint8_t byte;

	(void)arg;

	for (;;) {
		if (xQueueReceive(usblink_rx_queue, &byte, portMAX_DELAY) == pdTRUE)
			process_byte(byte);
	}
}

void usblink_tx_task(void *arg)
{
	atkp_frame_t frame;
	uint8_t buf[ATKP_FRAME_DATA_MAX + 5];
	uint8_t chk;
	uint8_t len;
	uint8_t i;

	(void)arg;

	for (;;) {
		if (xQueueReceive(g_usblink.tx_queue, &frame, portMAX_DELAY) != pdTRUE)
			continue;

		len = frame.data_len;
		if (len > ATKP_FRAME_DATA_MAX)
			len = ATKP_FRAME_DATA_MAX;

		/* --- Assemble ATKP frame --- */
		buf[0] = ATKP_START;
		buf[1] = ATKP_UP;
		buf[2] = frame.msg_id;
		buf[3] = len;

		for (i = 0; i < len; i++)
			buf[4 + i] = frame.data[i];

		/* XOR checksum over msg_id, data_len, and payload. */
		chk = buf[2] ^ buf[3];
		for (i = 0; i < len; i++)
			chk ^= frame.data[i];
		buf[4 + len] = chk;

		/* Retry while the CDC bulk-in endpoint is busy. */
		while (CDC_Transmit_FS(buf, (uint16_t)(5 + len)) == USBD_BUSY)
			osDelay(USBLINK_TX_RETRY_MS);

		/* Wait for transfer-complete callback to signal done. */
		xSemaphoreTake(usblink_tx_done, portMAX_DELAY);
	}
}

bool usblink_get_frame(atkp_frame_t *frame)
{
	if (!g_usblink.has_frame)
		return false;

	*frame = g_usblink.output;
	g_usblink.has_frame = false;
	return true;
}

bool usblink_send_frame(atkp_frame_t *frame)
{
	return xQueueSend(g_usblink.tx_queue, frame, 0) == pdTRUE;
}
