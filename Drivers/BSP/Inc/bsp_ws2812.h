// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board-level WS2812 RGB LED driver using TIM3 CH1 + DMA double-buffer.
 *
 * @details
 * This module provides low-level control of a WS2812 RGB LED ring (12 LEDs)
 * via PWM-based bit-banging on TIM3_CH1 with DMA-assisted transfer.
 *
 * Hardware Dependencies:
 * - TIM3 (general-purpose timer, 96 MHz clock)
 * - DMA1_Stream4, Channel 5 (memory-to-peripheral double-buffer mode)
 * - GPIO: PB4 (TIM3_CH1 PWM output), PB5 (module power enable),
 *   PB0 (headlight control)
 *
 * Thread Safety:
 * bsp_ws2812_send() is protected by an internal FreeRTOS binary semaphore
 * and blocks until the previous transfer completes. Concurrent calls from
 * multiple tasks are serialized. All control functions (power_set,
 * headlight_set) are safe to call from any context.
 *
 * Timing:
 * WS2812 protocol operates at 800 kHz carrier frequency with GRB color order.
 * Send completes in ~30 us per LED plus reset period (>60 us low).
 */

#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_WS2812_LED_COUNT 12

/**
 * @brief Initialize TIM3, DMA1_Stream4, GPIO, and FreeRTOS synchronization.
 *
 * @pre   FreeRTOS scheduler must be running (required for semaphore creation).
 *
 * @retval None
 *
 * @note  Safe to call multiple times; subsequent calls are no-ops.
 *        Must be called before any other bsp_ws2812_*() function.
 *
 * @warning Do not call from ISR context; uses FreeRTOS primitives.
 */
void bsp_ws2812_init(void);

/**
 * @brief Halt timer and DMA, releasing hardware resources.
 *
 * @retval None
 *
 * @note  Safe to call multiple times or before init().
 *        After calling this, bsp_ws2812_init() must be called again
 *        before the module can send data.
 */
void bsp_ws2812_deinit(void);

/**
 * @brief Send color data to the WS2812 LED chain via DMA transfer.
 *
 * @param[in] color  Pointer to array of [len][3] bytes in GRB order.
 *                   GRB means: [green_byte, red_byte, blue_byte].
 * @param[in] len    Number of LEDs to drive (1 to BSP_WS2812_LED_COUNT).
 *
 * @retval None
 *
 * @note  Blocks on internal semaphore until previous transfer completes.
 *        The function automatically appends two zero-frames after the last
 *        LED to satisfy the WS2812 >60 us reset low-pulse requirement.
 *        Returns silently if color==NULL or len<1.
 *
 * @warning Must not be called before bsp_ws2812_init().
 *          Do not modify the color array while this function is executing
 *          (until it returns).
 */
void bsp_ws2812_send(const uint8_t (*color)[3], uint16_t len);

/**
 * @brief Control the WS2812 module power enable pin (PB5).
 *
 * @param[in] on  true to assert PB5 (enable power), false to clear (disable).
 *
 * @retval None
 *
 * @note  This pin typically controls a power distribution switch.
 *        Safe to call from any context (ISR or task).
 */
void bsp_ws2812_power_set(bool on);

/**
 * @brief Control the headlight output GPIO (PB0).
 *
 * @param[in] on  true to assert PB0, false to clear.
 *
 * @retval None
 *
 * @note  Safe to call from any context (ISR or task).
 *        Typically used for front-facing LED or strobe output.
 */
void bsp_ws2812_headlight_set(bool on);

/**
 * @brief DMA transfer-complete ISR handler.
 *
 * @pre   Must be callable from DMA1_Stream4_IRQHandler with the module active.
 *
 * @retval None
 *
 * @note  This function is registered as the DMA callback but may also be
 *        called directly from the ISR dispatcher if needed.
 *
 * @warning Must only be called from ISR context (DMA1_Stream4_IRQHandler).
 */
void bsp_ws2812_dma_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_WS2812_H */
