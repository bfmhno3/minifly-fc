// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Buffered debug console output service.
 *
 * @details
 * Provides putchar/puts/write output to USART2 with a small software buffer.
 * Output is flushed on newline or when the buffer is full.
 * ISR-safe variant (console_service_putchar_from_isr) uses interrupt-driven
 * UART transmit instead of blocking HAL_UART_Transmit.
 */

#ifndef CONSOLE_SERVICE_H
#define CONSOLE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the console service.
 *
 * Creates the TX semaphore and clears the output buffer.
 */
void console_service_init(void);

/**
 * @brief  Self-test: returns true if the service was initialized.
 */
bool console_service_test(void);

/**
 * @brief  Write a single character to the console.
 *
 * Buffers the character; flushes to USART2 on newline or buffer-full.
 * Auto-detects ISR context and delegates to the ISR-safe variant.
 *
 * @param[in] ch  Character to write.
 * @return The character written, or EOF if service is not initialized.
 */
int console_service_putchar(int ch);

/**
 * @brief  ISR-safe variant of console_service_putchar.
 *
 * Uses xSemaphoreTakeFromISR and HAL_UART_Transmit_IT for non-blocking
 * output from interrupt context.
 *
 * @param[in] ch  Character to write.
 * @return The character written, or EOF if service is not initialized.
 */
int console_service_putchar_from_isr(int ch);

/**
 * @brief  Write a null-terminated string to the console.
 *
 * @param[in] str  String to write (must not be NULL).
 * @return Number of characters written, or EOF on error.
 */
int console_service_puts(const char *str);

/**
 * @brief  Write a fixed-length buffer to the console.
 *
 * @param[in] buf  Buffer to write (may contain embedded NULs).
 * @param[in] len  Number of bytes to write.
 */
void console_service_write(const char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif