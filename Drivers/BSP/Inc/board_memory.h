// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Flash memory map constants for the Minifly board.
 *
 * @details
 * This header defines the fixed flash region boundaries used by boot, config,
 * and application images. Values are macros so they can be shared by linker
 * scripts, startup code, and flash IAP routines without introducing a .c unit.
 *
 * Hardware assumption:
 * - Target MCU flash base address is provided by STM32 device headers.
 */

#ifndef BOARD_MEMORY_H
#define BOARD_MEMORY_H

#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Flash layout                                                       */
/* ------------------------------------------------------------------ */

/* Bootloader region size: first 16 KB from FLASH_BASE. */
#define BOARD_FLASH_BOOTLOADER_SIZE (16u * 1024u)

/* Configuration region size: next 16 KB after bootloader. */
#define BOARD_FLASH_CONFIG_SIZE (16u * 1024u)

/* Application offset from FLASH_BASE (bootloader + config). */
#define BOARD_FLASH_APP_OFFSET \
  (BOARD_FLASH_BOOTLOADER_SIZE + BOARD_FLASH_CONFIG_SIZE)

/* ------------------------------------------------------------------ */
/* Absolute region addresses                                          */
/* ------------------------------------------------------------------ */

/* Start address of configuration region. */
#define BOARD_FLASH_CONFIG_ADDR (FLASH_BASE + BOARD_FLASH_BOOTLOADER_SIZE)

/* Start address of application region. */
#define BOARD_FLASH_APP_ADDR (FLASH_BASE + BOARD_FLASH_APP_OFFSET)

/* ------------------------------------------------------------------ */
/* Factory-programmed MCU information (read-only)                     */
/* ------------------------------------------------------------------ */

/* Unique device ID base address (96-bit device identifier). */
#define BOARD_MCU_ID_ADDR 0x1FFF7A10UL

/* Flash size register address (device-reported flash capacity). */
#define BOARD_MCU_FLASH_SIZE_ADDR 0x1FFF7A22UL

#ifdef __cplusplus
}
#endif

#endif /* BOARD_MEMORY_H */
