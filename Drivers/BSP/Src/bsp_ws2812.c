// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief WS2812 RGB LED driver implementation using TIM3 PWM + DMA.
 *
 * @details
 * Drives a 12-LED RGB ring module via PB4 (TIM3_CH1 AF), with power control
 * on PB5 and headlight output on PB0. DMA1_Stream4 (channel 5) operates in
 * double-buffer circular mode to stream PWM values to the timer while the
 * current LED frame is being transmitted.
 *
 * Concurrency Model:
 * bsp_ws2812_send() acquires a binary semaphore before starting a transfer
 * and releases it only after the full chain (all LEDs + reset frames) has
 * been clocked out. The DMA interrupt callback (ws2812_dma_tc_cb) gives the
 * semaphore from ISR context via xSemaphoreGiveFromISR(). This ensures
 * only one transfer is active at a time.
 */

#include "bsp_ws2812.h"

#include <string.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

/*
 * WS2812 timing derived from 800 kHz PWM carrier (TIM3 runs at 96 MHz).
 * Each bit period is 1.25 us. Logic '1' requires 0.8 us high pulse,
 * logic '0' requires 0.3 us high pulse.
 *
 * Calculations:
 *   TIMING_ONE  = 0.8 us * 96 MHz / sec = 76.8 ticks, round to 80
 *   TIMING_ZERO = 0.3 us * 96 MHz / sec = 28.8 ticks, round to 30
 *   PWM_PERIOD  = 1.25 us * 96 MHz = 120 ticks, set to 119 (0-indexed)
 *
 * This achieves ~800 kHz carrier at 96 MHz system clock.
 * See WS2812 datasheet for detailed timings.
 */
#define TIMING_ONE 80  /* ~0.8 us high for logic '1' */
#define TIMING_ZERO 30 /* ~0.3 us high for logic '0' */
#define PWM_PERIOD 119 /* 96 MHz / 120 = 800 kHz carrier */

/*
 * Each WS2812 LED consumes 24 bits: 8 green + 8 red + 8 blue in GRB order.
 * Each bit maps to one PWM half-word value in the DMA buffer.
 */
#define BITS_PER_LED 24

/* --- GPIO pin definitions --- */
#define WS2812_DATA_PORT GPIOB
#define WS2812_DATA_PIN GPIO_PIN_4 /* TIM3_CH1 PWM output */

#define WS2812_POWER_PORT GPIOB
#define WS2812_POWER_PIN GPIO_PIN_5 /* Module power enable */

#define WS2812_HEADLIGHT_PORT GPIOB
#define WS2812_HEADLIGHT_PIN GPIO_PIN_0 /* Front LED or strobe */

/* --- DMA double buffers --- */
static uint16_t dma_buf0[BITS_PER_LED];
static uint16_t dma_buf1[BITS_PER_LED];

/* --- HAL handles --- */
static TIM_HandleTypeDef htim3;
static DMA_HandleTypeDef hdma_tim3_ch1;

/* --- State machine and synchronization --- */
static SemaphoreHandle_t transfer_done;
static volatile int current_led; /* Next LED index to be transmitted */
static volatile int total_leds;  /* Total LEDs in current send() call */
static const uint8_t (*volatile color_ptr)[3]; /* Pointer to color array */

static bool is_init;

/* forward declaration */
static void ws2812_dma_tc_cb(DMA_HandleTypeDef *hdma);

/* --- fill one LED's worth of PWM values into a DMA buffer --- */

/**
 * @brief Fill one LED's worth of PWM samples into a DMA buffer.
 *
 * Encodes a GRB color into 24 half-word PWM values in bit-serial order.
 * MSB-first transmission ensures correct bit mapping on the wire.
 *
 * @param[out] buf  Pointer to 24-element uint16_t buffer to fill.
 * @param[in]  grb  Pointer to 3-byte GRB color: grb[0]=green, grb[1]=red, grb[2]=blue.
 */
static void fill_led_pwm(uint16_t *buf, const uint8_t *grb)
{
  for (int i = 0; i < 8; i++)
    buf[i] = ((grb[0] << i) & 0x80) ? TIMING_ONE : TIMING_ZERO;
  for (int i = 0; i < 8; i++)
    buf[8 + i] = ((grb[1] << i) & 0x80) ? TIMING_ONE : TIMING_ZERO;
  for (int i = 0; i < 8; i++)
    buf[16 + i] = ((grb[2] << i) & 0x80) ? TIMING_ONE : TIMING_ZERO;
}

/* --- Hardware Initialization Functions --- */

/**
 * @brief Initialize GPIO pins for WS2812 module.
 *
 * Configures PB4 as TIM3_CH1 PWM output, and PB5, PB0 as push-pull outputs
 * with initial state low (powered off, headlight off).
 */
static void gpio_init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = { 0 };

  /* PB4: TIM3_CH1 AF output */
  gpio.Pin = WS2812_DATA_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(WS2812_DATA_PORT, &gpio);

  /* PB5: power enable, push-pull output, default low */
  gpio.Pin = WS2812_POWER_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0;
  HAL_GPIO_Init(WS2812_POWER_PORT, &gpio);
  HAL_GPIO_WritePin(WS2812_POWER_PORT, WS2812_POWER_PIN, GPIO_PIN_RESET);

  /* PB0: headlight, push-pull output, default low */
  gpio.Pin = WS2812_HEADLIGHT_PIN;
  HAL_GPIO_Init(WS2812_HEADLIGHT_PORT, &gpio);
  HAL_GPIO_WritePin(WS2812_HEADLIGHT_PORT, WS2812_HEADLIGHT_PIN,
                    GPIO_PIN_RESET);
}

/**
 * @brief Initialize TIM3 for PWM generation at 800 kHz.
 *
 * Configures TIM3 in PWM mode on channel 1 with period 120 ticks
 * to achieve 800 kHz carrier (96 MHz / 120 = 800 kHz).
 * PWM duty cycle is controlled by DMA updates to CCR1.
 */
static void tim3_init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0; /* No prescaling; use 96 MHz directly */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_PERIOD; /* 119 ticks = 800 kHz */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim3);

  TIM_OC_InitTypeDef oc = { 0 };
  oc.OCMode = TIM_OCMODE_PWM1; /* Output high for pulse, low after */
  oc.Pulse = 0;                /* CCR1 will be updated by DMA */
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);
}

/**
 * @brief Initialize DMA1_Stream4 for circular double-buffer transfers to TIM3_CCR1.
 *
 * Sets up DMA1_Stream4 (peripheral channel 5) in circular mode with two buffers
 * (dma_buf0, dma_buf1). When one buffer is being transferred to the timer, the
 * ISR handler fills the other buffer with the next LED's PWM data, providing
 * seamless pipelining.
 *
 * ISR priority is set to 9 (low) to avoid blocking time-critical tasks.
 */
static void dma_init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim3_ch1.Instance = DMA1_Stream4;
  hdma_tim3_ch1.Init.Channel = DMA_CHANNEL_5;
  hdma_tim3_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim3_ch1.Init.PeriphInc = DMA_PINC_DISABLE; /* CCR1 fixed address */
  hdma_tim3_ch1.Init.MemInc = DMA_MINC_ENABLE; /* Increment through buffer */
  hdma_tim3_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim3_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim3_ch1.Init.Mode = DMA_CIRCULAR; /* Restart after M0/M1 */
  hdma_tim3_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim3_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_tim3_ch1);

  /* register per-handle callback instead of global weak override */
  hdma_tim3_ch1.XferCpltCallback = ws2812_dma_tc_cb;

  /* Link DMA to timer: DMA feed TIM3_CCR1 via DMA_ID_CC1 */
  __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC1], hdma_tim3_ch1);

  /* Low priority (9) to avoid blocking time-critical ISRs */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

/* --- public API --- */

void bsp_ws2812_init(void)
{
  if (is_init)
    return;

  gpio_init();
  dma_init();
  tim3_init();

  /* Create binary semaphore for DMA transfer synchronization */
  if (transfer_done == NULL) {
    transfer_done = xSemaphoreCreateBinary();
    xSemaphoreGive(transfer_done); /* Start available */
  }

  current_led = 0;
  total_leds = 0;
  color_ptr = NULL;

  is_init = true;
}

void bsp_ws2812_deinit(void)
{
  if (!is_init)
    return;

  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_DMA_Abort(&hdma_tim3_ch1);

  is_init = false;
}
void bsp_ws2812_send(const uint8_t (*color)[3], uint16_t len)
{
  if (len < 1 || color == NULL)
    return;

  /* --- synchronization --- */
  /* Block until any in-flight transfer finishes. Protects the global
   * state (current_led, total_leds, color_ptr, dma_buf0, dma_buf1). */
  xSemaphoreTake(transfer_done, portMAX_DELAY);

  /* --- prepare state --- */
  current_led = 0;
  total_leds = len;
  color_ptr = color;

  /* --- prime double buffers --- */
  /* Pre-fill both buffers with the first two LEDs' data so the DMA
   * can start immediately without stalling. The ISR will refill them
   * as each transfer completes. */
  fill_led_pwm(dma_buf0, color_ptr[0]);
  current_led++;
  if (len > 1) {
    fill_led_pwm(dma_buf1, color_ptr[1]);
    current_led++;
  } else {
    /* If only 1 LED, fill dma_buf1 with zeros (reset period) */
    for (int i = 0; i < BITS_PER_LED; i++)
      dma_buf1[i] = 0;
  }

  /* --- start DMA transfer --- */
  /* Configure circular double-buffer mode: DMA alternates between dma_buf0
   * and dma_buf1, transferring BITS_PER_LED (24) half-words on each cycle. */
  HAL_DMAEx_MultiBufferStart_IT(&hdma_tim3_ch1, (uint32_t)dma_buf0,
                                (uint32_t)&htim3.Instance->CCR1,
                                (uint32_t)dma_buf1, BITS_PER_LED);

  /* Enable timer DMA request and start PWM generation */
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void bsp_ws2812_power_set(bool on)
{
  HAL_GPIO_WritePin(WS2812_POWER_PORT, WS2812_POWER_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void bsp_ws2812_headlight_set(bool on)
{
  HAL_GPIO_WritePin(WS2812_HEADLIGHT_PORT, WS2812_HEADLIGHT_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bsp_ws2812_dma_isr(void)
{
  HAL_DMA_IRQHandler(&hdma_tim3_ch1);
}

/* --- DMA transfer-complete callback (registered per-handle) --- */

/**
 * @brief DMA transfer-complete callback: refill idle buffer and manage sequence.
 *
 * Called from DMA ISR when a 24-halfword transfer (one LED's worth of data)
 * completes. This function:
 * 1. Identifies which buffer just finished (via DMA_SxCR_CT bit).
 * 2. Fills the now-idle buffer with the next LED's data, or zeros if all
 *    LEDs have been transmitted (for reset period).
 * 3. Detects end-of-sequence and releases the FreeRTOS semaphore to unblock
 *    bsp_ws2812_send().
 *
 * The WS2812 protocol requires >60 us of low pulse after the last LED bit.
 * We provide this by appending 2 zero-frames (48 us at 800 kHz) after
 * the last LED. Once these extra frames are transmitted, the transfer is done.
 */
static void ws2812_dma_tc_cb(DMA_HandleTypeDef *hdma)
{
  (void)hdma;

  BaseType_t wake = pdFALSE;

  /* --- determine which buffer just finished and refill the other --- */
  /* The DMA_SxCR_CT bit (current target) indicates which of M0/M1 is active.
   * If CT==0, we're currently in buf0, so buf1 just finished -> refill buf0.
   * If CT==1, we're currently in buf1, so buf0 just finished -> refill buf1. */
  if ((hdma_tim3_ch1.Instance->CR & DMA_SxCR_CT) == 0) {
    /* buf0 just finished, buf1 is active -> fill buf0 next */
    if (current_led < total_leds) {
      fill_led_pwm(dma_buf0, color_ptr[current_led]);
      current_led++;
    } else {
      /* All LEDs sent; append zero-frame for reset period */
      memset(dma_buf0, 0, sizeof(dma_buf0));
    }
  } else {
    /* Currently in buf1, so buf0 just finished; refill buf1 */
    if (current_led < total_leds) {
      fill_led_pwm(dma_buf1, color_ptr[current_led]);
      current_led++;
    } else {
      /* All LEDs sent; append zero-frame for reset period */
      memset(dma_buf1, 0, sizeof(dma_buf1));
    }
  }

  /* --- detect end of sequence --- */
  /* After all LEDs are sent, we append 2 zero-frames for reset period.
   * When current_led reaches total_leds + 2, all data has been transmitted. */
  if (current_led >= total_leds + 2) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_DMA_Abort(&hdma_tim3_ch1);
    total_leds = 0;

    /* Release semaphore to unblock bsp_ws2812_send(); safe in ISR context */
    xSemaphoreGiveFromISR(transfer_done, &wake);
    portYIELD_FROM_ISR(wake);
  }
}
