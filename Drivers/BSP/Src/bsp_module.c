// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package implementation for external module detect and power control.
 *
 * This module keeps the shared GPIO binding and ADC sampling details local to
 * the BSP layer. It does not implement module identification state machines or
 * module-specific bus switching policies.
 */
#include "bsp_module.h"

#include "adc.h"
#include "main.h"

#include <stdbool.h>
#include <stddef.h>

#define BSP_MODULE_ADC_SAMPLE_COUNT 10u
#define BSP_MODULE_ADC_TIMEOUT_MS 5u

#define BSP_MODULE_ADC_ID_LED_RING 2048u
#define BSP_MODULE_ADC_ID_WIFI_CAMERA 4095u
#define BSP_MODULE_ADC_ID_OPTICAL_FLOW 2815u
#define BSP_MODULE_ADC_ID_RESERVED_1 1280u
#define BSP_MODULE_ADC_TOLERANCE 50u

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} bsp_module_power_output_t;

static const bsp_module_power_output_t module_power_output = {
  MODULE_POWER_GPIO_Port,
  MODULE_POWER_Pin,
};

static bool module_initialized = false;

static void bsp_module_set_power_pin(bool on)
{
  HAL_GPIO_WritePin(module_power_output.port, module_power_output.pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void bsp_module_apply_safe_pin_state(void)
{
  bsp_module_set_power_pin(false);
}

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
