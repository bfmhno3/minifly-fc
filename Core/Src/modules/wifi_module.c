#include "modules/wifi_module.h"
#include "comm/commander.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#define WIFI_FRAME_LEN      8
#define WIFI_START_BYTE     0x66
#define WIFI_END_BYTE       0x99
#define WIFI_SCALE_ROLL     0.25f
#define WIFI_SCALE_PITCH    0.25f
#define WIFI_SCALE_YAW      1.6f
#define WIFI_RAW_CENTER     128.0f
#define WIFI_THRUST_SHIFT   8
#define WIFI_THRUST_MID     32768
#define WIFI_THRUST_LOW     10000
#define WIFI_THRUST_ZERO    256
#define WIFI_TASK_STACK     150
#define WIFI_TASK_PRIO      4
#define WIFI_RX_TIMEOUT_MS  1000

enum wifi_rx_state {
	WIFI_RX_WAIT_START = 0,
	WIFI_RX_WAIT_DATA,
	WIFI_RX_WAIT_CHKSUM,
	WIFI_RX_WAIT_END,
};

static bool is_init;
static TaskHandle_t task_handle;
static uint16_t last_thrust;

static bool wifi_data_crc(const uint8_t *data)
{
	uint8_t xor_val = data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5];
	return xor_val == data[6];
}

static void wifi_cmd_process(uint8_t cmd)
{
	bool key_flight  = (cmd >> 0) & 1;
	bool key_land    = (cmd >> 1) & 1;
	bool emer_stop   = (cmd >> 2) & 1;
	bool flight_mode = (cmd >> 4) & 1;

	if (commander_get_ctrl_mode() == 1) {
		if (key_flight) {
			commander_set_key_flight(true);
			commander_set_key_land(false);
		}
		if (key_land) {
			commander_set_key_flight(false);
			commander_set_key_land(true);
		}
		if (emer_stop) {
			commander_set_key_flight(false);
			commander_set_key_land(false);
			commander_set_emer_stop(true);
		} else {
			commander_set_emer_stop(false);
		}
	} else {
		commander_set_ctrl_mode(0);
		commander_set_key_flight(false);
		commander_set_key_land(false);
	}

	commander_set_flight_mode(flight_mode);
}

static void wifi_data_handle(const uint8_t *data)
{
	ctrl_val_t val;

	val.roll  = ((float)data[1] - WIFI_RAW_CENTER) * WIFI_SCALE_ROLL;
	val.pitch = ((float)data[2] - WIFI_RAW_CENTER) * WIFI_SCALE_PITCH;
	val.yaw   = ((float)data[4] - WIFI_RAW_CENTER) * WIFI_SCALE_YAW;
	val.thrust = (uint16_t)data[3] << WIFI_THRUST_SHIFT;
	val.trim_pitch = 0.0f;
	val.trim_roll  = 0.0f;

	if (val.thrust == WIFI_THRUST_MID && last_thrust < WIFI_THRUST_LOW) {
		commander_set_ctrl_mode(1);
		commander_set_key_flight(false);
		commander_set_key_land(false);
	} else if (val.thrust == 0 && last_thrust > WIFI_THRUST_ZERO) {
		commander_set_ctrl_mode(0);
		val.thrust = 0;
	}
	last_thrust = val.thrust;

	wifi_cmd_process(data[5]);
	commander_cache_ctrl_data(CTRL_SRC_WIFI, &val);
}

static void wifi_rx_task(void *param)
{
	(void)param;

	uint8_t raw[WIFI_FRAME_LEN];
	uint8_t byte;
	uint8_t data_idx;
	enum wifi_rx_state state;

	for (;;) {
		state = WIFI_RX_WAIT_START;
		data_idx = 0;

		for (;;) {
			if (HAL_UART_Receive(&huart1, &byte, 1, WIFI_RX_TIMEOUT_MS) != HAL_OK) {
				break;
			}

			switch (state) {
			case WIFI_RX_WAIT_START:
				if (byte == WIFI_START_BYTE) {
					raw[0] = byte;
					data_idx = 1;
					state = WIFI_RX_WAIT_DATA;
				}
				break;

			case WIFI_RX_WAIT_DATA:
				raw[data_idx] = byte;
				data_idx++;
				if (data_idx == 6) {
					state = WIFI_RX_WAIT_CHKSUM;
				}
				break;

			case WIFI_RX_WAIT_CHKSUM:
				raw[6] = byte;
				if (wifi_data_crc(raw)) {
					state = WIFI_RX_WAIT_END;
				} else {
					state = WIFI_RX_WAIT_START;
				}
				break;

			case WIFI_RX_WAIT_END:
				if (byte == WIFI_END_BYTE) {
					raw[7] = byte;
					wifi_data_handle(raw);
				}
				state = WIFI_RX_WAIT_START;
				break;

			default:
				state = WIFI_RX_WAIT_START;
				break;
			}
		}
	}
}

void wifi_module_init(void)
{
	if (is_init) {
		if (task_handle != NULL) {
			vTaskResume(task_handle);
		}
		return;
	}

	last_thrust = 0;

	xTaskCreate(wifi_rx_task, "WIFILINK", WIFI_TASK_STACK,
	            NULL, WIFI_TASK_PRIO, &task_handle);

	is_init = true;
}

void wifi_module_deinit(void)
{
	if (task_handle != NULL) {
		vTaskSuspend(task_handle);
	}
}
