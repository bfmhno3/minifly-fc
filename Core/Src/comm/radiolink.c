#include "comm/radiolink.h"

#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "services/ledseq.h"

#define RADIOLINK_RX_QUEUE_BYTES 1024
#define RADIOLINK_TX_QUEUE_FRAMES 30
#define RADIOLINK_TASK_POLL_MS 10

#define ATKP_START 0xAA
#define ATKP_UP 0xAA
#define ATKP_DOWN 0xAF

enum rx_state {
	STATE_START1,
	STATE_START2,
	STATE_MSG_ID,
	STATE_DATA_LEN,
	STATE_DATA,
	STATE_CHKSUM,
};

enum {
	TX_IDLE,
	TX_BUSY,
};

struct radiolink {
	QueueHandle_t rx_queue;
	QueueHandle_t tx_queue;
	SemaphoreHandle_t tx_done;
	StaticTask_t task_buf;
	StackType_t task_stack[128];

	enum rx_state state;
	radio_frame_t rx_frame;
	uint8_t data_index;
	uint8_t checksum;
	uint8_t cmd;

	radio_frame_t output;
	bool has_frame;

	volatile uint8_t tx_state;
	bool is_init;
};

static struct radiolink g_rl;

#define DMA_SXCR_EN ((uint32_t)0x00000001)

static void nrf_dma_pause(void)
{
	DMA1_Stream6->CR &= ~DMA_SXCR_EN;
}

static void nrf_dma_resume(void)
{
	DMA1_Stream6->CR |= DMA_SXCR_EN;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != USART2)
		return;

	g_rl.tx_state = TX_IDLE;

	BaseType_t woken = pdFALSE;
	xSemaphoreGiveFromISR(g_rl.tx_done, &woken);

	ledseq_run(LEDSEQ_LED_DATA_TX, LEDSEQ_PATTERN_LINKUP);

	portYIELD_FROM_ISR(woken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t byte;

	if (huart->Instance != USART2)
		return;

	BaseType_t woken = pdFALSE;
	xQueueSendFromISR(g_rl.rx_queue, &byte, &woken);

	HAL_UART_Receive_IT(&huart2, &byte, 1);

	portYIELD_FROM_ISR(woken);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if (pin != GPIO_PIN_0)
		return;

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
		nrf_dma_pause();
	else
		nrf_dma_resume();
}

static bool send_frame_dma(void)
{
	static uint8_t buf[RADIOLINK_FRAME_DATA_MAX + 5];
	radio_frame_t frame;
	uint8_t host_chk;
	uint8_t len;
	uint8_t i;

	if (g_rl.tx_state != TX_IDLE)
		return false;

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
		return false;

	if (xQueueReceive(g_rl.tx_queue, &frame, 0) != pdTRUE)
		return false;

	len = frame.data_len;
	if (len > RADIOLINK_FRAME_DATA_MAX)
		len = RADIOLINK_FRAME_DATA_MAX;

	buf[0] = ATKP_START;
	buf[1] = ATKP_DOWN;
	buf[2] = frame.msg_id;
	buf[3] = len;

	for (i = 0; i < len; i++)
		buf[4 + i] = frame.data[i];

	host_chk = buf[2] ^ buf[3];
	for (i = 0; i < len; i++)
		host_chk ^= frame.data[i];
	buf[4 + len] = host_chk;

	g_rl.tx_state = TX_BUSY;

	if (HAL_UART_Transmit_DMA(&huart2, buf, 5 + len) != HAL_OK) {
		g_rl.tx_state = TX_IDLE;
		return false;
	}

	xSemaphoreTake(g_rl.tx_done, pdMS_TO_TICKS(100));

	return true;
}

static void process_byte(uint8_t byte)
{
	switch (g_rl.state) {
	case STATE_START1:
		if (byte == ATKP_START)
			g_rl.state = STATE_START2;
		break;

	case STATE_START2:
		if (byte == ATKP_UP || byte == ATKP_DOWN) {
			g_rl.cmd = byte;
			g_rl.state = STATE_MSG_ID;
		} else if (byte != ATKP_START) {
			g_rl.state = STATE_START1;
		}
		break;

	case STATE_MSG_ID:
		g_rl.rx_frame.msg_id = byte;
		g_rl.checksum = byte;
		g_rl.state = STATE_DATA_LEN;
		break;

	case STATE_DATA_LEN:
		g_rl.rx_frame.data_len = byte;
		g_rl.checksum ^= byte;

		if (byte > RADIOLINK_FRAME_DATA_MAX) {
			g_rl.state = STATE_START1;
		} else if (byte == 0) {
			g_rl.state = STATE_CHKSUM;
		} else {
			g_rl.data_index = 0;
			g_rl.state = STATE_DATA;
		}
		break;

	case STATE_DATA:
		g_rl.rx_frame.data[g_rl.data_index] = byte;
		g_rl.checksum ^= byte;
		g_rl.data_index++;

		if (g_rl.data_index >= g_rl.rx_frame.data_len)
			g_rl.state = STATE_CHKSUM;
		break;

	case STATE_CHKSUM:
		if (byte == g_rl.checksum) {
			g_rl.output = g_rl.rx_frame;
			g_rl.has_frame = true;

			ledseq_run(LEDSEQ_LED_DATA_RX, LEDSEQ_PATTERN_LINKUP);
		}
		g_rl.state = STATE_START1;
		break;
	}
}

void radiolink_init(void)
{
	uint8_t byte = 0;

	if (g_rl.is_init)
		return;

	g_rl.rx_queue = xQueueCreate(RADIOLINK_RX_QUEUE_BYTES, sizeof(uint8_t));
	g_rl.tx_queue = xQueueCreate(RADIOLINK_TX_QUEUE_FRAMES, sizeof(radio_frame_t));
	g_rl.tx_done = xSemaphoreCreateBinary();

	g_rl.state = STATE_START1;
	g_rl.has_frame = false;
	g_rl.tx_state = TX_IDLE;
	g_rl.is_init = true;

	HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_UART_Receive_IT(&huart2, &byte, 1);
}

void radiolink_task(void *arg)
{
	uint8_t byte;

	(void)arg;

	for (;;) {
		if (xQueueReceive(g_rl.rx_queue, &byte,
				  pdMS_TO_TICKS(RADIOLINK_TASK_POLL_MS)) == pdTRUE)
			process_byte(byte);
		else
			send_frame_dma();
	}
}

bool radiolink_get_frame(radio_frame_t *frame)
{
	if (!g_rl.has_frame)
		return false;

	*frame = g_rl.output;
	g_rl.has_frame = false;
	return true;
}
