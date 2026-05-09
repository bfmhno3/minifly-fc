// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package implementation for external module detect and power control.
 *
 * @details
 * This implementation layer encapsulates STM32 HAL-specific GPIO and ADC operations.
 * It provides a clean abstraction boundary, keeping hardware details local and
 * allowing higher-layer module management code to remain platform-agnostic.
 *
 * [Hardware Bindings]
 * - GPIO: MODULE_POWER_GPIO_Port / MODULE_POWER_Pin (defined in main.h/stm32*xx_hal_conf.h)
 * - ADC: hadc1 (global ADC1 handle, configured in adc.c)
 * - Sampling pin: PB1 / ADC1_IN9 (module detect voltage divider)
 *
 * [Initialization Contract]
 * All public APIs (bsp_module_detect_read_raw, bsp_module_power_set) check
 * the module_initialized flag. This "fail-closed" pattern prevents accidental
 * use before HAL setup is complete.
 *
 * [ADC Threshold Mapping]
 * Raw ADC readings are calibrated against resistor dividers on each module variant.
 * See bsp_module_detect_read_raw() header for the complete mapping table.
 * Threshold classification logic resides in the higher-level module manager, NOT here.
 */

#include "bsp_module.h"

#include <stdbool.h>
#include <stddef.h>

#include "adc.h"
#include "main.h"

#define BSP_MODULE_ADC_SAMPLE_COUNT \
  10u // Number of ADC samples to average per read burst
#define BSP_MODULE_ADC_TIMEOUT_MS \
  5u // Polling timeout for single ADC conversion (ms)

// ADC threshold values (raw 12-bit codes) for each module variant.
// These are center points for module identification; hysteresis must be applied
// at the higher layer to prevent glitching during transient power events.
#define BSP_MODULE_ADC_ID_LED_RING 2048u
#define BSP_MODULE_ADC_ID_WIFI_CAMERA 4095u
#define BSP_MODULE_ADC_ID_OPTICAL_FLOW 2815u
#define BSP_MODULE_ADC_ID_RESERVED_1 1280u
#define BSP_MODULE_ADC_TOLERANCE \
  50u // Tolerance band around each threshold (unused in this layer)

typedef struct bsp_module_power_output {
  GPIO_TypeDef *port;
  uint16_t pin;
} bsp_module_power_output_t;

static const bsp_module_power_output_t module_power_output = {
  MODULE_POWER_GPIO_Port,
  MODULE_POWER_Pin,
};

static bool module_initialized = false;

/**
 * @brief Write the module power pin to a desired GPIO state.
 *
 * @param[in] on true -> GPIO_PIN_SET, false -> GPIO_PIN_RESET
 *
 * @details
 * This is the single point of exit for all power control. Centralizing GPIO writes
 * here eases future transitions to interrupt-driven or DMA-driven power sequencing.
 */
static void bsp_module_set_power_pin(bool on)
{
  HAL_GPIO_WritePin(module_power_output.port, module_power_output.pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Apply the safe (power-off) state to the module power pin.
 *
 * @details
 * Used during initialization and error recovery to ensure the external module
 * power rail is disabled. This prevents uncontrolled current draw or power-on glitches.
 */
static void bsp_module_apply_safe_pin_state(void)
{
  bsp_module_set_power_pin(false);
}

/**
 * @brief Trigger a single software-polled ADC conversion and return the result.
 *
 * @param[out] sample Pointer to uint16_t where the 12-bit ADC result will be stored.
 *
 * @retval true  Conversion succeeded; *sample contains valid ADC data.
 * @retval false Conversion failed (NULL pointer, HAL error, or timeout).
 *
 * @details
 * Performs one complete ADC1 polling cycle (start -> poll -> read -> stop).
 * This is a low-level primitive; callers should collect multiple samples
 * and average them to reduce noise.
 */
static bool bsp_module_sample_once(uint16_t *sample)
{
  if (sample == NULL) {
    return false;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return false;
  }

  if (HAL_ADC_PollForConversion(&hadc1, BSP_MODULE_ADC_TIMEOUT_MS) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc1);
    return false;
  }

  *sample = (uint16_t)HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_Stop(&hadc1) != HAL_OK) {
    return false;
  }

  return true;
}

/**
 * @brief Collect and average a burst of ADC samples from the module detect pin.
 *
 * @return Averaged 12-bit ADC value [0, 4095], or 0 if any sample fails.
 *
 * @details
 * Takes BSP_MODULE_ADC_SAMPLE_COUNT independent conversions, sums them, and
 * returns the integer mean. This averaging reduces high-frequency noise from
 * switching transients on shared power supply rails without introducing
 * excessive latency (typical burst time < 100 ms).
 *
 * NOTE: If any single conversion fails, the function returns 0 immediately.
 * This fail-fast behavior prevents partial results from being misinterpreted
 * as a valid (albeit noisy) module detection value.
 */
static uint16_t bsp_module_average_samples(void)
{
  uint32_t sum = 0u;
  uint32_t index = 0u;

  for (; index < BSP_MODULE_ADC_SAMPLE_COUNT; ++index) {
    uint16_t sample = 0u;

    if (!bsp_module_sample_once(&sample)) {
      return 0u;
    }

    sum += sample;
  }

  return (uint16_t)(sum / BSP_MODULE_ADC_SAMPLE_COUNT);
}

void bsp_module_init(void)
{
  bsp_module_apply_safe_pin_state();
  module_initialized = true;
}

uint16_t bsp_module_detect_read_raw(void)
{
  if (!module_initialized) {
    return 0u;
  }

  return bsp_module_average_samples();
}

void bsp_module_power_set(bool on)
{
  if (!module_initialized) {
    return;
  }

  bsp_module_set_power_pin(on);
}
