// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Flash memory layout constants for the Minifly board.
 *
 * All addresses and sizes are expressed as preprocessor defines so they can
 * be used in linker scripts, vector table relocation, and flash read/write
 * routines without requiring a .c translation unit.
 */
#ifndef BOARD_MEMORY_H
#define BOARD_MEMORY_H

#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bootloader region: 16 KB starting at FLASH_BASE */
#define BOARD_FLASH_BOOTLOADER_SIZE  (16u * 1024u)

/* Config parameter region: 16 KB immediately after bootloader */
#define BOARD_FLASH_CONFIG_SIZE      (16u * 1024u)

/* Combined offset from FLASH_BASE to the application region */
#define BOARD_FLASH_APP_OFFSET       (BOARD_FLASH_BOOTLOADER_SIZE + \
                                      BOARD_FLASH_CONFIG_SIZE)

/* Absolute addresses */
#define BOARD_FLASH_CONFIG_ADDR      (FLASH_BASE + BOARD_FLASH_BOOTLOADER_SIZE)
#define BOARD_FLASH_APP_ADDR         (FLASH_BASE + BOARD_FLASH_APP_OFFSET)

/* MCU device signature addresses (read-only, factory-programmed) */
#define BOARD_MCU_ID_ADDR            0x1FFF7A10UL
#define BOARD_MCU_FLASH_SIZE_ADDR    0x1FFF7A22UL

#ifdef __cplusplus
}
#endif

#endif /* BOARD_MEMORY_H */
