#include "bsp_ws2812.h"

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "semphr.h"

/*
 * WS2812 timing at 800 kHz PWM carrier (TIM3 clock = 96 MHz):
 *   TIMING_ONE  = 0.8 us -> 96 * 0.8 = 76.8, round to 80
 *   TIMING_ZERO = 0.3 us -> 96 * 0.3 = 28.8, round to 30
 */
#define TIMING_ONE   80
#define TIMING_ZERO  30
#define PWM_PERIOD   119   /* 96 MHz / 120 = 800 kHz */

/* per-LED: 8 bits G + 8 bits R + 8 bits B = 24 PWM half-words */
#define BITS_PER_LED  24

/* --- GPIO pin definitions --- */
#define WS2812_DATA_PORT      GPIOB
#define WS2812_DATA_PIN       GPIO_PIN_4   /* TIM3_CH1 */

#define WS2812_POWER_PORT     GPIOB
#define WS2812_POWER_PIN      GPIO_PIN_5

#define WS2812_HEADLIGHT_PORT GPIOB
#define WS2812_HEADLIGHT_PIN  GPIO_PIN_0

/* --- DMA double buffers --- */
static uint16_t dma_buf0[BITS_PER_LED];
static uint16_t dma_buf1[BITS_PER_LED];

static TIM_HandleTypeDef htim3;
static DMA_HandleTypeDef hdma_tim3_ch1;

static SemaphoreHandle_t transfer_done;
static volatile int current_led;
static volatile int total_leds;
static const uint8_t (*volatile color_ptr)[3];

static bool is_init;

/* forward declaration */
static void ws2812_dma_tc_cb(DMA_HandleTypeDef *hdma);

/* --- fill one LED's worth of PWM values into a DMA buffer --- */

static void fill_led_pwm(uint16_t *buf, const uint8_t *grb)
{
    for (int i = 0; i < 8; i++)
        buf[i]      = ((grb[0] << i) & 0x80) ? TIMING_ONE : TIMING_ZERO;
    for (int i = 0; i < 8; i++)
        buf[8 + i]  = ((grb[1] << i) & 0x80) ? TIMING_ONE : TIMING_ZERO;
    for (int i = 0; i < 8; i++)
        buf[16 + i] = ((grb[2] << i) & 0x80) ? TIMING_ONE : TIMING_ZERO;
}

/* --- TIM3 + DMA hardware init --- */

static void gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* PB4: TIM3_CH1 AF output */
    gpio.Pin       = WS2812_DATA_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(WS2812_DATA_PORT, &gpio);

    /* PB5: power enable, push-pull output, default low */
    gpio.Pin       = WS2812_POWER_PIN;
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(WS2812_POWER_PORT, &gpio);
    HAL_GPIO_WritePin(WS2812_POWER_PORT, WS2812_POWER_PIN, GPIO_PIN_RESET);

    /* PB0: headlight, push-pull output, default low */
    gpio.Pin = WS2812_HEADLIGHT_PIN;
    HAL_GPIO_Init(WS2812_HEADLIGHT_PORT, &gpio);
    HAL_GPIO_WritePin(WS2812_HEADLIGHT_PORT, WS2812_HEADLIGHT_PIN,
                      GPIO_PIN_RESET);
}

static void tim3_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = PWM_PERIOD;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);
}

static void dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_tim3_ch1.Instance                 = DMA1_Stream4;
    hdma_tim3_ch1.Init.Channel             = DMA_CHANNEL_5;
    hdma_tim3_ch1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim3_ch1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim3_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim3_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_tim3_ch1.Init.Mode                = DMA_CIRCULAR;
    hdma_tim3_ch1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_tim3_ch1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_tim3_ch1);

    /* register per-handle callback instead of global weak override */
    hdma_tim3_ch1.XferCpltCallback = ws2812_dma_tc_cb;

    __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC1], hdma_tim3_ch1);

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

    if (transfer_done == NULL) {
        transfer_done = xSemaphoreCreateBinary();
        xSemaphoreGive(transfer_done);
    }

    current_led = 0;
    total_leds  = 0;
    color_ptr   = NULL;

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

    /* wait for any in-flight transfer to finish */
    xSemaphoreTake(transfer_done, portMAX_DELAY);

    current_led = 0;
    total_leds  = len;
    color_ptr   = color;

    /* pre-fill first two LEDs into double buffers */
    fill_led_pwm(dma_buf0, color_ptr[0]);
    current_led++;
    if (len > 1) {
        fill_led_pwm(dma_buf1, color_ptr[1]);
        current_led++;
    } else {
        for (int i = 0; i < BITS_PER_LED; i++)
            dma_buf1[i] = 0;
    }

    /* configure DMA double-buffer for 24 half-words per transfer */
    HAL_DMAEx_MultiBufferStart_IT(&hdma_tim3_ch1,
                                  (uint32_t)dma_buf0,
                                  (uint32_t)&htim3.Instance->CCR1,
                                  (uint32_t)dma_buf1,
                                  BITS_PER_LED);

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

static void ws2812_dma_tc_cb(DMA_HandleTypeDef *hdma)
{
    (void)hdma;

    BaseType_t wake = pdFALSE;

    /*
     * Determine which buffer was just consumed and fill the other one
     * with the next LED's data. When all LEDs are sent, write zeros
     * for the reset period (>60 us low) then stop.
     */
    if (__HAL_DMA_GET_CURRENT_TARGET(&hdma_tim3_ch1) == DMA_TARGET_0) {
        /* buf0 just finished, buf1 is active -> fill buf0 next */
        if (current_led < total_leds) {
            fill_led_pwm(dma_buf0, color_ptr[current_led]);
            current_led++;
        } else {
            memset(dma_buf0, 0, sizeof(dma_buf0));
        }
    } else {
        /* buf1 just finished, buf0 is active -> fill buf1 next */
        if (current_led < total_leds) {
            fill_led_pwm(dma_buf1, color_ptr[current_led]);
            current_led++;
        } else {
            memset(dma_buf1, 0, sizeof(dma_buf1));
        }
    }

    /* after 2 extra zero frames the reset period is satisfied */
    if (current_led >= total_leds + 2) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_DMA_Abort(&hdma_tim3_ch1);
        total_leds = 0;
        xSemaphoreGiveFromISR(transfer_done, &wake);
        portYIELD_FROM_ISR(wake);
    }
}
