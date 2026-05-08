// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Radio transport implementation -- nRF24L01 via USART2.
 *
 * @details
 * RX path: HAL UART RX IT -> isr pushes bytes into rx_queue -> radiolink_task
 *          dequeues and feeds process_byte() state machine -> completed frame
 *          stored in output buffer for radiolink_get_frame().
 *
 * TX path: radiolink_send_frame() enqueues into tx_queue -> radiolink_task
 *          calls send_frame_dma() on idle -> DMA transmit via USART2 ->
 *          HAL_UART_TxCpltCallback signals tx_done semaphore.
 *
 * Flow control: EXTI0 on PA0 pauses/resumes DMA when the nRF24L01
 *               deasserts/asserts its CE line.
 */

#include "comm/radiolink.h"

#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"

#include "main.h"
#include "usart.h"
#include "services/ledseq.h"

/** @brief RX byte queue depth. */
#define RADIOLINK_RX_QUEUE_BYTES 1024

/** @brief TX frame queue depth. */
#define RADIOLINK_TX_QUEUE_FRAMES 30

/** @brief Idle poll interval when no RX bytes arrive (ms). */
#define RADIOLINK_TASK_POLL_MS 10

/* ATKP framing constants (duplicated here to avoid depending on atkp.h). */
#define ATKP_START 0xAA
#define ATKP_UP 0xAA
#define ATKP_DOWN 0xAF

/** @brief RX frame parser states. */
typedef enum rx_state {
  STATE_START1,
  STATE_START2,
  STATE_MSG_ID,
  STATE_DATA_LEN,
  STATE_DATA,
  STATE_CHKSUM,
} rx_state_t;

/** @brief TX DMA state. */
typedef enum tx_state {
  TX_IDLE,
  TX_BUSY,
} tx_state_t;

/**
 * @brief  Runtime context for the radio transport module.
 *
 * @details
 * Groups queue/semaphore handles, parser scratch state, and TX/RX handoff
 * buffers into one owner object so task and ISR communication points are
 * explicit and easy to audit.
 *
 * Concurrency model:
 * - Task context owns parser progression and queue consumption/production.
 * - ISR context signals RX/TX events through FreeRTOS primitives.
 * - tx_state is volatile because it is shared across task and ISR contexts.
 */
typedef struct radiolink {
  QueueHandle_t rx_queue;      // Byte stream from UART RX ISR to parser task.
  QueueHandle_t tx_queue;      // Outbound frames waiting for DMA transmission.
  SemaphoreHandle_t tx_done;   // TX-complete signal posted by UART TX ISR.
  StaticTask_t task_buf;       // Storage for statically created radiolink task.
  StackType_t task_stack[128]; // Task stack (words), sized for parser+DMA loop.

  rx_state_t state;       // Current state of ATKP byte parser FSM.
  radio_frame_t rx_frame; // Frame under construction before checksum passes.
  uint8_t data_index;     // Payload write index while in STATE_DATA
  uint8_t checksum;       // Running XOR checksum for current frame.
  uint8_t cmd;            // Direction byte (ATKP_UP/ATKP_DOWN) from header.

  radio_frame_t output; // Last fully validated frame for public retrieval.
  bool has_frame;       // Latch indicating output contains unread frame.

  volatile uint8_t tx_state; // TX_IDLE/TX_BUSY gate shared with TX complete ISR.
  bool is_init;              // One-time init guard to prevent double setup.
} radiolink_t;

static radiolink_t g_rl;

/* --- nRF24L01 DMA flow control via PA0 / EXTI0 --- */

/** @brief DMA Stream6 CR.EN bit. */
#define DMA_SXCR_EN ((uint32_t)0x00000001)

/** @brief Pause USART2 TX DMA -- called when nRF24L01 signals busy (PA0 high). */
static void nrf_dma_pause(void)
{
  DMA1_Stream6->CR &= ~DMA_SXCR_EN;
}

/** @brief Resume USART2 TX DMA -- called when nRF24L01 signals ready (PA0 low). */
static void nrf_dma_resume(void)
{
  DMA1_Stream6->CR |= DMA_SXCR_EN;
}

/* --- HAL callbacks (ISR context) --- */

/**
 * @brief  USART2 TX-complete callback.
 *
 * Signals the TX-done semaphore so send_frame_dma() can proceed
 * to the next frame.  Blinks the data-TX LED.
 */
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

/**
 * @brief  USART2 RX-complete callback.
 *
 * Pushes the received byte into the RX queue and re-arms the interrupt.
 */
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

/**
 * @brief  EXTI callback -- nRF24L01 flow control on PA0.
 *
 * When PA0 goes high the nRF24L01 is not ready to receive, so we pause
 * the DMA; when it goes low we resume.
 */
void radiolink_exti_callback(uint16_t pin)
{
  if (pin != GPIO_PIN_0)
    return;

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    nrf_dma_pause();
  else
    nrf_dma_resume();
}

/* --- TX path --- */

/**
 * @brief  Send one queued frame via DMA if the bus is idle.
 *
 * Assembles the ATKP frame (start + direction + msg_id + len + data + checksum)
 * into a static buffer and initiates DMA transfer on USART2.
 *
 * @return true if a frame was sent, false if bus busy, radio busy, or queue empty.
 */
static bool send_frame_dma(void)
{
  static uint8_t buf[RADIOLINK_FRAME_DATA_MAX + 5];
  radio_frame_t frame;
  uint8_t host_chk;
  uint8_t len;
  uint8_t i;

  if (g_rl.tx_state != TX_IDLE)
    return false;

  /* PA0 high means nRF24L01 cannot accept data. */
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    return false;

  if (xQueueReceive(g_rl.tx_queue, &frame, 0) != pdTRUE)
    return false;

  len = frame.data_len;
  if (len > RADIOLINK_FRAME_DATA_MAX)
    len = RADIOLINK_FRAME_DATA_MAX;

  /* --- Assemble ATKP frame --- */
  buf[0] = ATKP_START;
  buf[1] = ATKP_DOWN;
  buf[2] = frame.msg_id;
  buf[3] = len;

  for (i = 0; i < len; i++)
    buf[4 + i] = frame.data[i];

  /* XOR checksum over msg_id, data_len, and payload. */
  host_chk = buf[2] ^ buf[3];
  for (i = 0; i < len; i++)
    host_chk ^= frame.data[i];
  buf[4 + len] = host_chk;

  g_rl.tx_state = TX_BUSY;

  if (HAL_UART_Transmit_DMA(&huart2, buf, 5 + len) != HAL_OK) {
    g_rl.tx_state = TX_IDLE;
    return false;
  }

  /* Block until DMA transfer completes (signaled from ISR). */
  xSemaphoreTake(g_rl.tx_done, pdMS_TO_TICKS(100));

  return true;
}

/* --- RX path: byte-level frame parser --- */

/**
 * @brief  Feed one byte into the RX state machine.
 *
 * On checksum match the decoded frame is copied to the output buffer
 * and has_frame is set.  On any error the state resets to START1.
 */
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

/* --- Public API --- */

void radiolink_init(void)
{
  uint8_t byte = 0;

  if (g_rl.is_init)
    return;

  g_rl.rx_queue = xQueueCreate(RADIOLINK_RX_QUEUE_BYTES, sizeof(uint8_t));
  g_rl.tx_queue =
    xQueueCreate(RADIOLINK_TX_QUEUE_FRAMES, sizeof(radio_frame_t));
  g_rl.tx_done = xSemaphoreCreateBinary();

  g_rl.state = STATE_START1;
  g_rl.has_frame = false;
  g_rl.tx_state = TX_IDLE;
  g_rl.is_init = true;

  /* EXTI0 priority 6: lower than UART ISR (5) but above FreeRTOS tasks. */
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

bool radiolink_send_frame(const radio_frame_t *frame)
{
  return xQueueSend(g_rl.tx_queue, frame, 0) == pdTRUE;
}

bool radiolink_get_frame(radio_frame_t *frame)
{
  if (!g_rl.has_frame)
    return false;

  *frame = g_rl.output;
  g_rl.has_frame = false;
  return true;
}
