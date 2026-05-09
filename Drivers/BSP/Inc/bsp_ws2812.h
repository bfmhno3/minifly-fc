// SPDX-License-Identifier: MIT

#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Board-level WS2812 RGB LED driver using TIM3 CH1 + DMA.
 *
 * Hardware: PB4 (TIM3_CH1), PB5 (power enable), PB0 (headlight).
 * DMA: DMA1_Stream4 / Channel 5, double-buffer mode.
 */

#define BSP_WS2812_LED_COUNT 12

/**
 * @brief Initialize TIM3, DMA, and GPIO for WS2812 output.
 *
 * Must be called after module power-on. Safe to call multiple times;
 * subsequent calls are no-ops.
 */
void bsp_ws2812_init(void);

/**
 * @brief Deinitialize WS2812 peripheral and release DMA.
 */
void bsp_ws2812_deinit(void);

/**
 * @brief Send color data to the WS2812 chain via DMA.
 *
 * @param color  Array of [len][3] in GRB order (green, red, blue).
 * @param len    Number of LEDs to drive.
 *
 * This call blocks until the previous transfer completes.
 */
void bsp_ws2812_send(const uint8_t (*color)[3], uint16_t len);

/**
 * @brief Control the WS2812 module power pin (PB5).
 */
void bsp_ws2812_power_set(bool on);

/**
 * @brief Control the headlight GPIO (PB0).
 */
void bsp_ws2812_headlight_set(bool on);

/**
 * @brief DMA transfer-complete ISR handler.
 *
 * Call from DMA1_Stream4_IRQHandler when the LED ring module is active.
 */
void bsp_ws2812_dma_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_WS2812_H */
