// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Buffered debug console output service implementation.
 *
 * @details
 * Uses a 30-byte software buffer with a binary semaphore for mutual exclusion.
 * The buffer is flushed to USART2 on newline or when full.
 * ISR path uses HAL_UART_Transmit_IT (non-blocking); task path uses
 * HAL_UART_Transmit (blocking).
 */

#include "services/console_service.h"

#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"

#define CONSOLE_BUFFER_SIZE 30

static uint8_t tx_buffer[CONSOLE_BUFFER_SIZE];
static uint8_t buffer_len = 0;
static StaticSemaphore_t sem_buffer;
static SemaphoreHandle_t tx_sem = NULL;
static bool isInit = false;

/**
 * @brief  Flush the software buffer to USART2 (blocking).
 */
static void console_flush(void)
{
    if (buffer_len > 0)
    {
        HAL_UART_Transmit(&huart2, tx_buffer, buffer_len, HAL_MAX_DELAY);
        buffer_len = 0;
    }
}

/** @brief  See console_service.h */
void console_service_init(void)
{
    if (isInit)
    {
        return;
    }

    tx_sem = xSemaphoreCreateBinaryStatic(&sem_buffer);
    if (tx_sem != NULL)
    {
        xSemaphoreGive(tx_sem);
    }

    memset(tx_buffer, 0, sizeof(tx_buffer));
    buffer_len = 0;
    isInit = true;
}

/** @brief  See console_service.h */
bool console_service_test(void)
{
    return isInit;
}

/** @brief  See console_service.h */
int console_service_putchar(int ch)
{
    if (!isInit)
    {
        return EOF;
    }

    bool isInInterrupt = (__get_IPSR() != 0);
    if (isInInterrupt)
    {
        return console_service_putchar_from_isr(ch);
    }

    if (xSemaphoreTake(tx_sem, portMAX_DELAY) == pdTRUE)
    {
        if (buffer_len < CONSOLE_BUFFER_SIZE)
        {
            tx_buffer[buffer_len] = (uint8_t)ch;
            buffer_len++;
        }

        if ((ch == '\n') || (buffer_len >= CONSOLE_BUFFER_SIZE))
        {
            console_flush();
        }

        xSemaphoreGive(tx_sem);
    }

    return ch;
}

/** @brief  See console_service.h */
int console_service_putchar_from_isr(int ch)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (!isInit)
    {
        return EOF;
    }

    if (xSemaphoreTakeFromISR(tx_sem, &higherPriorityTaskWoken) == pdTRUE)
    {
        if (buffer_len < CONSOLE_BUFFER_SIZE)
        {
            tx_buffer[buffer_len] = (uint8_t)ch;
            buffer_len++;
        }

        if ((ch == '\n') || (buffer_len >= CONSOLE_BUFFER_SIZE))
        {
            HAL_UART_Transmit_IT(&huart2, tx_buffer, buffer_len);
            buffer_len = 0;
        }

        xSemaphoreGiveFromISR(tx_sem, &higherPriorityTaskWoken);
    }

    return ch;
}

/** @brief  See console_service.h */
int console_service_puts(const char *str)
{
    int ret = 0;

    if (str == NULL)
    {
        return EOF;
    }

    while (*str)
    {
        if (console_service_putchar(*str++) == EOF)
        {
            return EOF;
        }
        ret++;
    }

    return ret;
}

/** @brief  See console_service.h */
void console_service_write(const char *buf, size_t len)
{
    size_t i;

    if (!isInit || (buf == NULL) || (len == 0))
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        console_service_putchar(buf[i]);
    }
}