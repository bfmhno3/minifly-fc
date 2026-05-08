// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  LED sequence animation engine.
 *
 * @details
 * Drives LED blink patterns using FreeRTOS software timers.  Each pattern
 * is an array of ledseq_step entries that specify on/off values and timing.
 * Patterns are priority-indexed (lower enum value = higher priority) so
 * a high-priority pattern (e.g., low battery) preempts lower ones.
 *
 * Multiple LEDs can run independent sequences simultaneously, each backed
 * by its own FreeRTOS timer.
 */

#ifndef LEDSEQ_H
#define LEDSEQ_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_led.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Step action codes
 * @{ */
#define LEDSEQ_WAIT_MS(x) (x) /**< Hold current state for x milliseconds */
#define LEDSEQ_STOP (-1)      /**< Stop this pattern (release LED) */
#define LEDSEQ_LOOP (-2)      /**< Jump back to the first step */
/** @} */

/**
 * @brief  A single step in an LED animation sequence.
 */
typedef struct ledseq_step {
  bool value; /**< LED on (true) or off (false) */
  int action; /**< LEDSEQ_WAIT_MS(n), LEDSEQ_STOP, or LEDSEQ_LOOP */
} ledseq_step_t;

/**
 * @brief  Available LED blink patterns, ordered by priority (lowest index = highest priority).
 */
typedef enum ledseq_pattern {
  LEDSEQ_PATTERN_LOWBAT = 0, /**< Low battery warning */
  LEDSEQ_PATTERN_CHARGED,    /**< Charge complete */
  LEDSEQ_PATTERN_CHARGING,   /**< Actively charging */
  LEDSEQ_PATTERN_CALIBRATED, /**< Gyro calibration done */
  LEDSEQ_PATTERN_ALIVE,      /**< Heartbeat indicator */
  LEDSEQ_PATTERN_LINKUP,     /**< Communication link established */
  LEDSEQ_PATTERN_COUNT       /**< Number of patterns (sentinel) */
} ledseq_pattern_t;

/** @name LED-to-pattern mapping
 * @{ */
#define LEDSEQ_LED_SYS BSP_LED_GREEN_R     /**< System status LED */
#define LEDSEQ_LED_LOWBAT BSP_LED_RED_R    /**< Low battery LED */
#define LEDSEQ_LED_CHG BSP_LED_BLUE_L      /**< Charge status LED */
#define LEDSEQ_LED_DATA_RX BSP_LED_GREEN_L /**< Data receive LED */
#define LEDSEQ_LED_DATA_TX BSP_LED_RED_L   /**< Data transmit LED */
#define LEDSEQ_LED_ERR1 BSP_LED_RED_L      /**< Error LED 1 */
#define LEDSEQ_LED_ERR2 BSP_LED_RED_R      /**< Error LED 2 */
/** @} */

/**
 * @brief  Initialize the LED sequence engine.
 *
 * Creates per-LED timers and the access semaphore.
 */
void ledseq_init(void);

/**
 * @brief  Start (or restart) a pattern on a given LED.
 *
 * @param[in] target   LED index (BSP_LED_*).
 * @param[in] pattern  Pattern to run (ledseq_pattern enum value).
 */
void ledseq_run(uint8_t target, uint8_t pattern);

/**
 * @brief  Stop all patterns on a given LED (turns it off).
 *
 * @param[in] target  LED index (BSP_LED_*).
 */
void ledseq_stop(uint8_t target);

/**
 * @brief  Globally enable or disable all LED sequences.
 *
 * @param[in] enable  true to enable, false to suppress all LED output.
 */
void ledseq_enable(bool enable);

/**
 * @brief  Self-test: returns true if init completed.
 */
bool ledseq_test(void);

#ifdef __cplusplus
}
#endif

#endif
