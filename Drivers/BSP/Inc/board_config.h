// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Compile-time hardware feature configuration for the Minifly board.
 *
 * @details
 * Centralizes board identity, clock, PWM, LED, expansion-module ADC references,
 * and optional feature flags as read-only macros.
 *
 * Design notes:
 * - All values are compile-time constants and must stay side-effect free.
 * - Feature flags here are build-time hints, not runtime detection results.
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
/*  Clock                                                             */
/* ------------------------------------------------------------------ */

/* System clock in Hz (HSE 8 MHz, PLL x12 -> 96 MHz). */
#define BOARD_SYSCLK_HZ 96000000UL

/* ------------------------------------------------------------------ */
/*  PWM / Motor                                                       */
/* ------------------------------------------------------------------ */

/* Number of motor PWM outputs available on this board. */
#define BOARD_MOTOR_COUNT 4u

/* PWM timer period (8-bit range: 0..255). */
#define BOARD_PWM_PERIOD 255u

/* Timer input clock for PWM generation in Hz. */
#define BOARD_PWM_CLOCK_HZ 96000000UL

/* ------------------------------------------------------------------ */
/* Status LEDs                                                        */
/* ------------------------------------------------------------------ */

/* Number of onboard status LEDs. */
#define BOARD_LED_COUNT 5u

/* ------------------------------------------------------------------ */
/*  Expansion module ADC identification                               */
/* ------------------------------------------------------------------ */

/* ADC reference value for LED ring module divider (R1:R2 = 10K:10K). */
#define BOARD_MODULE_ADC_LED_RING 2048u

/* ADC reference value for WiFi camera module divider (R1 = 10K). */
#define BOARD_MODULE_ADC_WIFI_CAMERA 4095u

/* ADC reference value for optical flow module divider (R1:R2 = 10K:22K). */
#define BOARD_MODULE_ADC_OPTICAL_FLOW 2815u

/* ADC reference value for reserved module slot 1 (R1:R2 = 22K:10K). */
#define BOARD_MODULE_ADC_RESERVED_1 1280u

/* Allowed ADC deviation when matching module signatures. */
#define BOARD_MODULE_ADC_TOLERANCE 50u

/* ------------------------------------------------------------------ */
/*  Optional feature flags (compile-time hints, not runtime state)    */
/* ------------------------------------------------------------------ */

/* 1 if USB CDC link is available on this board variant. */
#define BOARD_HAS_USB_LINK 1

/* 1 if optical flow module is permanently populated. */
#define BOARD_HAS_OPTICAL_FLOW 0

/* 1 if WiFi camera module is permanently populated. */
#define BOARD_HAS_WIFI_CAMERA 0

/* 1 if LED ring module is permanently populated. */
#define BOARD_HAS_LED_RING 0

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
