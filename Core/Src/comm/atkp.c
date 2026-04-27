#include "comm/atkp.h"
#include "comm/radiolink.h"
#include "comm/usblink.h"
#include "services/console_service.h"
#include "cmsis_os.h"

#define ATKP_RX_QUEUE_SIZE 10
#define ATKP_TX_PERIOD_MS   1

#define ATKP_PERIOD_STATUS     30
#define ATKP_PERIOD_SENSOR     10
#define ATKP_PERIOD_RCDATA     40
#define ATKP_PERIOD_POWER      100
#define ATKP_PERIOD_MOTOR      40
#define ATKP_PERIOD_SENSOR2    40
#define ATKP_PERIOD_SPEED      50
#define ATKP_PERIOD_USER_DATA  20

static QueueHandle_t rx_queue;
static bool is_init;

static bool radiolink_send(const atkp_frame_t *frame)
{
	radio_frame_t rf;
	rf.msg_id = frame->msg_id;
	rf.data_len = frame->data_len;
	for (uint8_t i = 0; i < frame->data_len; i++)
		rf.data[i] = frame->data[i];
	return radiolink_send_frame(&rf);
}

static void send_frame_dual(const atkp_frame_t *frame)
{
	radiolink_send(frame);
	usblink_send_frame(frame);
}

static void fill_with(uint8_t *data, uint8_t *len, uint8_t size)
{
	*len = size;
	for (uint8_t i = 0; i < size; i++)
		data[i] = 0;
}

static void send_periodic(void)
{
	static uint32_t tick;
	atkp_frame_t frame;

	tick += ATKP_TX_PERIOD_MS;

	if (tick % ATKP_PERIOD_STATUS == 0) {
		frame.msg_id = ATKP_UP_STATUS;
		fill_with(frame.data, &frame.data_len, 27);
		send_frame_dual(&frame);
	}

	if (tick % ATKP_PERIOD_SENSOR == 0) {
		frame.msg_id = ATKP_UP_SENSOR;
		fill_with(frame.data, &frame.data_len, 26);
		send_frame_dual(&frame);
	}

	if (tick % ATKP_PERIOD_RCDATA == 0) {
		frame.msg_id = ATKP_UP_RCDATA;
		fill_with(frame.data, &frame.data_len, 18);
		send_frame_dual(&frame);
	}

	if (tick % ATKP_PERIOD_POWER == 0) {
		frame.msg_id = ATKP_UP_POWER;
		fill_with(frame.data, &frame.data_len, 13);
		send_frame_dual(&frame);
	}

	if (tick % ATKP_PERIOD_MOTOR == 0) {
		frame.msg_id = ATKP_UP_MOTOR;
		fill_with(frame.data, &frame.data_len, 8);
		send_frame_dual(&frame);
	}

	if (tick % ATKP_PERIOD_SENSOR2 == 0) {
		frame.msg_id = ATKP_UP_SENSOR2;
		fill_with(frame.data, &frame.data_len, 4);
		send_frame_dual(&frame);
	}

	if (tick % ATKP_PERIOD_SPEED == 0) {
		frame.msg_id = ATKP_UP_SPEED;
		fill_with(frame.data, &frame.data_len, 12);
		send_frame_dual(&frame);
	}

	if (tick % ATKP_PERIOD_USER_DATA == 0) {
		frame.msg_id = ATKP_UP_USER_DATA1;
		fill_with(frame.data, &frame.data_len, 2);
		send_frame_dual(&frame);
	}
}

void atkp_init(void)
{
	if (is_init)
		return;

	rx_queue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_frame_t));
	is_init = true;
}

void atkp_tx_task(void *arg)
{
	(void)arg;

	for (;;) {
		send_periodic();
		osDelay(ATKP_TX_PERIOD_MS);
	}
}

void atkp_rx_task(void *arg)
{
	atkp_frame_t frame;

	(void)arg;

	for (;;) {
		if (radiolink_get_frame((radio_frame_t *)&frame)) {
			xQueueSend(rx_queue, &frame, 0);
		} else if (usblink_get_frame(&frame)) {
			xQueueSend(rx_queue, &frame, 0);
		}

		osDelay(1);
	}
}

bool atkp_receive_packet(atkp_frame_t *frame)
{
	return xQueueReceive(rx_queue, frame, 0) == pdTRUE;
}
