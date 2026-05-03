// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Internal flash read/write/erase abstraction for the Minifly board.
 *
 * @details
 * All addresses must be word-aligned (4 bytes).  Writes and erases are
 * restricted to the application region -- the bootloader and config parameter
 * regions are protected by address-range checks.
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mark the flash driver as initialized.
 *
 * @note Must be called before any other flash API.
 */
void bsp_flash_init(void);

/**
 * @brief Check whether an address range is valid and writable.
 *
 * @param[in] address  Start address (must be word-aligned).
 * @param[in] length   Number of bytes (must be a multiple of 4).
 *
 * @retval true  Range is within the writable application region.
 * @retval false Range is invalid, unaligned, or overlaps protected regions.
 */
bool bsp_flash_is_range_valid(uint32_t address, size_t length);

/**
 * @brief Read data from internal flash into a buffer.
 *
 * @param[in]  address  Source flash address (word-aligned).
 * @param[out] buffer   Destination buffer (must be at least @p length bytes).
 * @param[in]  length   Number of bytes to read (multiple of 4).
 *
 * @retval true  Read succeeded.
 * @retval false Invalid arguments or range check failed.
 */
bool bsp_flash_read(uint32_t address, void *buffer, size_t length);

/**
 * @brief Erase one or more flash sectors covering the given range.
 *
 * @param[in] address  Start address (word-aligned, must lie in app region).
 * @param[in] length   Number of bytes to erase (multiple of 4).
 *
 * @retval true  Erase succeeded.
 * @retval false Invalid range, HAL erase failure, or unlock failure.
 *
 * @warning Erasing destroys all data in the affected sectors.  The erase
 * granularity is per-sector (16 KB for sectors 0-3, 64 KB for sector 4,
 * 128 KB for sectors 5-7 on STM32F411).
 */
bool bsp_flash_erase(uint32_t address, size_t length);

/**
 * @brief Program data into internal flash (word-by-word).
 *
 * @param[in] address  Destination flash address (word-aligned).
 * @param[in] data     Source buffer (must be at least @p length bytes).
 * @param[in] length   Number of bytes to write (multiple of 4).
 *
 * @retval true  Write succeeded.
 * @retval false Invalid range, HAL program failure, or unlock failure.
 *
 * @pre The target region must be erased before writing.
 */
bool bsp_flash_write(uint32_t address, const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
