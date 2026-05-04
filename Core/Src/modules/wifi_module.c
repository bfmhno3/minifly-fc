// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  WiFi RC receiver expansion module implementation.
 *
 * @details
 * Receives 8-byte frames over UART1 from a WiFi RC transmitter:
 *   [0x66] [roll] [pitch] [thrust] [yaw] [cmd] [xor_chksum] [0x99]
 *
 * Stick bytes are unsigned 0..255 centered at 128.  The command byte
 * encodes flight/land/stop buttons as individual bits.  Thrust == 0x8000
 * with low previous thrust triggers mode switch to RC; thrust == 0 with
 * high previous thrust switches back to idle.
 */

#include "modules/wifi_module.h"
#include "comm/commander.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#define WIFI_FRAME_LEN      8       /**< total frame length in bytes */
#define WIFI_START_BYTE     0x66    /**< frame start marker */
#define WIFI_END_BYTE       0x99    /**< frame end marker */
#define WIFI_SCALE_ROLL     0.25f   /**< raw-to-degrees scale for roll */
#define WIFI_SCALE_PITCH    0.25f   /**< raw-to-degrees scale for pitch */
#define WIFI_SCALE_YAW      1.6f    /**< raw-to-degrees scale for yaw */
#define WIFI_RAW_CENTER     128.0f  /**< center value for unsigned stick bytes */
#define WIFI_THRUST_SHIFT   8       /**< shift raw thrust byte to 16-bit range */
#define WIFI_THRUST_MID     32768   /**< thrust value that signals mode switch to RC */
#define WIFI_THRUST_LOW     10000   /**< previous thrust threshold for mode switch */
#define WIFI_THRUST_ZERO    256     /**< thrust value that signals mode switch to idle */
#define WIFI_TASK_STACK     150     /**< FreeRTOS task stack size in words */
#define WIFI_TASK_PRIO      4       /**< FreeRTOS task priority */
#define WIFI_RX_TIMEOUT_MS  1000    /**< UART receive timeout in milliseconds */

enum wifi_rx_state {
	WIFI_RX_WAIT_START = 0,
	WIFI_RX_WAIT_DATA,
	WIFI_RX_WAIT_CHKSUM,
	WIFI_RX_WAIT_END,
};

static bool is_init;
static TaskHandle_t task_handle;
static uint16_t last_thrust;

/**
 * @brief  Validate the XOR checksum of a received frame.
 *
 * Checksum covers bytes [1..5] and is stored in byte [6].
 */
static bool wifi_data_crc(const uint8_t *data)
{
	uint8_t xor_val = data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5];
	return xor_val == data[6];
}

/**
 * @brief  Process the command byte from a WiFi frame.
 *
 * Decodes flight/land/stop button bits and updates commander state.
 * Only acts on button commands when ctrl_mode == 1 (RC active).
 */
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

/**
 * @brief  Convert a validated WiFi frame into commander control values.
 *
 * Translates raw stick bytes to roll/pitch/yaw degrees and thrust.
 * Handles mode switching: thrust at 0x8000 (stick pushed up) with low
 * previous thrust switches to RC mode; thrust at 0 with high previous
 * thrust switches back to idle.
 */
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

/**
 * @brief  FreeRTOS task: receives WiFi frames over UART1.
 *
 * Implements a simple state machine: waits for the start byte (0x66),
 * collects 5 data bytes + 1 checksum byte, validates the XOR checksum,
 * then waits for the end byte (0x99).  On timeout or framing error,
 * resets to the idle state.
 */
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
