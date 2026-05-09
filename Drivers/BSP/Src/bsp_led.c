// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief Board support package implementation for on-board status LEDs.
 *
 * The implementation keeps the GPIO binding, electrical polarity, and logical
 * LED state tracking local to the BSP layer.
 */
#include "bsp_led.h"

#include "main.h"

#include <stdbool.h>

typedef enum {
  BSP_LED_ACTIVE_HIGH = 0,
  BSP_LED_ACTIVE_LOW = 1,
} bsp_led_polarity_t;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  bsp_led_polarity_t polarity;
} bsp_led_output_t;

static const bsp_led_output_t led_outputs[BSP_LED_COUNT] = {
  { LED_BLUE_L_GPIO_Port, LED_BLUE_L_Pin, BSP_LED_ACTIVE_HIGH },
  { LED_GREEN_L_GPIO_Port, LED_GREEN_L_Pin, BSP_LED_ACTIVE_LOW },
  { LED_RED_L_GPIO_Port, LED_RED_L_Pin, BSP_LED_ACTIVE_LOW },
  { LED_GREEN_R_GPIO_Port, LED_GREEN_R_Pin, BSP_LED_ACTIVE_LOW },
  { LED_RED_R_GPIO_Port, LED_RED_R_Pin, BSP_LED_ACTIVE_LOW },
};

static bool leds_initialized = false;
static bool led_states[BSP_LED_COUNT] = { false };

static bool bsp_led_is_valid_id(uint8_t id)
{
  return id < BSP_LED_COUNT;
}

static GPIO_PinState bsp_led_get_pin_state(const bsp_led_output_t *output,
                                           bool on)
{
  if (output->polarity == BSP_LED_ACTIVE_LOW) {
    return on ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  return on ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static void bsp_led_write(uint8_t id, bool on)
{
  const bsp_led_output_t *output = &led_outputs[id];

  HAL_GPIO_WritePin(output->port, output->pin,
                    bsp_led_get_pin_state(output, on));
  led_states[id] = on;
}

void bsp_led_init(void)
{
  uint8_t index = 0u;

  for (; index < BSP_LED_COUNT; ++index) {
    bsp_led_write(index, false);
  }

  leds_initialized = true;
}

void bsp_led_set(uint8_t id, bool on)
{
  if (!leds_initialized || !bsp_led_is_valid_id(id)) {
    return;
  }

  bsp_led_write(id, on);
}

void bsp_led_toggle(uint8_t id)
{
  if (!leds_initialized || !bsp_led_is_valid_id(id)) {
    return;
  }

  bsp_led_write(id, !led_states[id]);
}
