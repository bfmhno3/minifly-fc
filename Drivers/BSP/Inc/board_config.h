// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Hardware feature configuration for the Minifly board.
 *
 * Centralizes board-level constants that were previously scattered across BSP
 * and platform source files.  All defines are read-only compile-time constants.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Board identity                                                     */
/* ------------------------------------------------------------------ */

#define BOARD_NAME "Minifly"
#define BOARD_MCU "STM32F411CEUx"

/* ------------------------------------------------------------------ */
/*  Clock                                                              */
/* ------------------------------------------------------------------ */

/** System clock frequency in Hz (HSE 8 MHz, PLL x12 -> 96 MHz). */
#define BOARD_SYSCLK_HZ 96000000UL

/* ------------------------------------------------------------------ */
/*  PWM / Motor                                                        */
/* ------------------------------------------------------------------ */

/** Number of motor outputs. */
#define BOARD_MOTOR_COUNT 4u

/** PWM timer period (8-bit: 0-255). */
#define BOARD_PWM_PERIOD 255u

/** Timer clock feeding the PWM prescaler. */
#define BOARD_PWM_CLOCK_HZ 96000000UL

/* ------------------------------------------------------------------ */
/*  LEDs                                                               */
/* ------------------------------------------------------------------ */

/** Number of on-board status LEDs. */
#define BOARD_LED_COUNT 5u

/* ------------------------------------------------------------------ */
/*  Expansion module ADC identification                                */
/* ------------------------------------------------------------------ */

/** ADC reference value for the LED Ring module (R1:R2 = 10K:10K). */
#define BOARD_MODULE_ADC_LED_RING 2048u

/** ADC reference value for the WiFi Camera module (R1 = 10K). */
#define BOARD_MODULE_ADC_WIFI_CAMERA 4095u

/** ADC reference value for the Optical Flow module (R1:R2 = 10K:22K). */
#define BOARD_MODULE_ADC_OPTICAL_FLOW 2815u

/** ADC reference value for reserved module slot 1 (R1:R2 = 22K:10K). */
#define BOARD_MODULE_ADC_RESERVED_1 1280u

/** Acceptable ADC deviation from the reference value. */
#define BOARD_MODULE_ADC_TOLERANCE 50u

/* ------------------------------------------------------------------ */
/*  Optional feature flags (compile-time hints, not runtime state)     */
/* ------------------------------------------------------------------ */

/** Set to 1 if USB CDC link is available on this board. */
#define BOARD_HAS_USB_LINK 1

/** Set to 1 if an optical flow module is permanently mounted. */
#define BOARD_HAS_OPTICAL_FLOW 0

/** Set to 1 if a WiFi camera module is permanently mounted. */
#define BOARD_HAS_WIFI_CAMERA 0

/** Set to 1 if an LED ring module is permanently mounted. */
#define BOARD_HAS_LED_RING 0

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
