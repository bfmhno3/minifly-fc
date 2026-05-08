// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  WS2812 LED ring expansion module implementation.
 *
 * @details
 * Each visual effect is a static function that writes RGB values into a
 * shared buffer.  A FreeRTOS software timer fires every 50 ms, calls the
 * active effect function, and pushes the buffer to the WS2812 chain via
 * bsp_ws2812_send().
 *
 * Color ordering throughout this file is GRB (WS2812 native), not RGB.
 */

#include "modules/ledring_module.h"

#include <string.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "modules/module_manager.h"
#include "bsp_ws2812.h"
#include "control/stabilizer.h"
#include "control/flight_types.h"
#include "services/sensors_types.h"

#define NBR_LEDS BSP_WS2812_LED_COUNT

#define LIMIT(a) ((a) > 255 ? 255 : ((a) < 0 ? 0 : (a)))
#define SIGN(a) ((a) >= 0 ? 1 : -1)
#define COPY_RGB(dst, src) \
  do {                     \
    (dst)[0] = (src)[0];   \
    (dst)[1] = (src)[1];   \
    (dst)[2] = (src)[2];   \
  } while (0)

typedef void (*ledring_effect_fn)(uint8_t buf[][3], bool reset);

static bool is_init;
static bool headlight_on;
static TimerHandle_t timer_handle;
static volatile ledring_effect_t current_effect;
static volatile ledring_effect_t requested_effect;

/* --- color palette (GRB order) --- */

static const uint8_t color_black[3] = { 0, 0, 0 };
static const uint8_t color_blue[3] = { 0, 0, 255 };

/**
 * @brief  Effect: all LEDs off.
 */
static void effect_off(uint8_t buf[][3], bool reset)
{
  (void)reset;
  for (int i = 0; i < NBR_LEDS; i++)
    COPY_RGB(buf[i], color_black);
  headlight_on = false;
}

/* --- effect: color test (9 colors cycling) --- */

#define COLOR_TEST_NUM 9
#define COLOR_TEST_DELAY \
  10 /**< ticks between color changes (500 ms at 50 ms/tick) */

static const uint8_t test_palette[COLOR_TEST_NUM][3] = {
  { 0, 0, 0 },    /* off */
  { 32, 32, 32 }, /* white */
  { 0, 64, 0 },   /* red (GRB) */
  { 64, 0, 0 },   /* green */
  { 0, 0, 64 },   /* blue */
  { 64, 64, 0 },  /* yellow */
  { 0, 64, 64 },  /* magenta */
  { 64, 0, 64 },  /* cyan */
  { 0, 64, 0 },   /* red again for visual rhythm */
};

/**
 * @brief  Effect: cycles through test_palette, toggles headlight periodically.
 */
static void effect_color_test(uint8_t buf[][3], bool reset)
{
  static uint8_t color_idx;
  static uint8_t tick_cnt;
  static uint8_t hl_cnt;
  static bool hl_state;

  if (reset) {
    color_idx = 0;
    tick_cnt = 0;
    hl_cnt = 0;
    hl_state = false;
  }

  for (int i = 0; i < NBR_LEDS; i++)
    COPY_RGB(buf[i], test_palette[color_idx]);

  if (++tick_cnt >= COLOR_TEST_DELAY) {
    tick_cnt = 0;
    color_idx = (color_idx + 1) % COLOR_TEST_NUM;
  }

  if (++hl_cnt >= COLOR_TEST_DELAY * 3) {
    hl_cnt = 0;
    hl_state = !hl_state;
  }
  headlight_on = hl_state;
}

/* ---- effect: attitude induce (pitch/roll -> LED brightness) ---- */

#define ATT_MIDDLE 10 /**< baseline brightness at level hover */
#define ATT_LIMIT 20  /**< clamp angle in degrees */

/**
 * @brief  Effect: maps pitch/roll to directional LED brightness.
 *
 * Front LEDs show pitch (red channel), side LEDs show roll (blue channel).
 * Squaring with sign preserves direction while giving a non-linear feel
 * -- small corrections are subtle, large tilts are dramatic.
 */
static void effect_attitude(uint8_t buf[][3], bool reset)
{
  if (reset) {
    for (int i = 0; i < NBR_LEDS; i++)
      memset(buf[i], 0, 3);
  }

  attitude_t att;
  stabilizer_get_attitude(&att);

  float pitch = -att.pitch;
  float roll = att.roll;

  /* clamp to +/- 20 degrees */
  pitch = (pitch > ATT_LIMIT) ? ATT_LIMIT :
                                (pitch < -ATT_LIMIT ? -ATT_LIMIT : pitch);
  roll = (roll > ATT_LIMIT) ? ATT_LIMIT :
                              (roll < -ATT_LIMIT ? -ATT_LIMIT : roll);

  /* square with sign for non-linear response */
  pitch = SIGN(pitch) * pitch * pitch;
  roll = SIGN(roll) * roll * roll;

  /* front 3 LEDs (index 8,9,10): red channel, positive pitch */
  for (int i = 8; i <= 10; i++)
    buf[i][0] = LIMIT(ATT_MIDDLE + (int)pitch);

  /* back 3 LEDs (index 2,3,4): red channel, negative pitch */
  for (int i = 2; i <= 4; i++)
    buf[i][0] = LIMIT(ATT_MIDDLE - (int)pitch);

  /* right 3 LEDs (index 5,6,7): blue channel, negative roll */
  for (int i = 5; i <= 7; i++)
    buf[i][2] = LIMIT(ATT_MIDDLE - (int)roll);

  /* left 3 LEDs (index 11,0,1): blue channel, positive roll */
  buf[11][2] = LIMIT(ATT_MIDDLE + (int)roll);
  buf[0][2] = LIMIT(ATT_MIDDLE + (int)roll);
  buf[1][2] = LIMIT(ATT_MIDDLE + (int)roll);
}

/* ---- effect: gyro induce (angular rate -> RGB) ---- */

#define GYRO_MAX_RATE 512 /**< clamp gyro reading to this range (deg/s) */

/**
 * @brief  Effect: maps gyroscope angular rates to RGB channels.
 *
 * gz -> R, gy -> G, gx -> B.  Each axis is clamped to +/- GYRO_MAX_RATE
 * and scaled to 0..255.
 */
static void effect_gyro(uint8_t buf[][3], bool reset)
{
  (void)reset;

  sensor_data_t sd;
  stabilizer_get_sensor_data(&sd);

  int gx = (int)sd.gyro.x;
  int gy = (int)sd.gyro.y;
  int gz = (int)sd.gyro.z;

  gx = (gx > GYRO_MAX_RATE) ? GYRO_MAX_RATE :
                              (gx < -GYRO_MAX_RATE ? -GYRO_MAX_RATE : gx);
  gy = (gy > GYRO_MAX_RATE) ? GYRO_MAX_RATE :
                              (gy < -GYRO_MAX_RATE ? -GYRO_MAX_RATE : gy);
  gz = (gz > GYRO_MAX_RATE) ? GYRO_MAX_RATE :
                              (gz < -GYRO_MAX_RATE ? -GYRO_MAX_RATE : gz);

  /* scale to 0..255 */
  uint8_t r = (uint8_t)LIMIT(SIGN(gz) * gz / 2);
  uint8_t g = (uint8_t)LIMIT(SIGN(gy) * gy / 2);
  uint8_t b = (uint8_t)LIMIT(SIGN(gx) * gx / 2);

  for (int i = 0; i < NBR_LEDS; i++) {
    buf[i][0] = r;
    buf[i][1] = g;
    buf[i][2] = b;
  }
}

/* ---- effect: blue blink ---- */

/**
 * @brief  Effect: periodic blue blink (50% duty, 1-second period).
 */
static void effect_blink(uint8_t buf[][3], bool reset)
{
  static int tick;

  if (reset)
    tick = 0;

  if ((tick < 10) && (tick & 1)) {
    for (int i = 0; i < NBR_LEDS; i++)
      COPY_RGB(buf[i], color_blue);
  } else {
    for (int i = 0; i < NBR_LEDS; i++)
      COPY_RGB(buf[i], color_black);
  }

  if (++tick >= 20)
    tick = 0;
}

/* ---- effect: flashlight (all white) ---- */

/**
 * @brief  Effect: all LEDs full white, headlight on.
 */
static void effect_flashlight(uint8_t buf[][3], bool reset)
{
  (void)reset;
  static const uint8_t white[3] = { 255, 255, 255 };
  for (int i = 0; i < NBR_LEDS; i++)
    COPY_RGB(buf[i], white);
  headlight_on = true;
}

/* ---- effect: breathing LED (cycling R/G/B combinations) ---- */

#define BREATH_MAX 30    /**< peak brightness level */
#define BREATH_STEP 3.0f /**< brightness increment per tick */

/**
 * @brief  Effect: breathing ramp cycling through R, G, B channel combinations.
 *
 * channel_mask is a 3-bit field (bit0=R, bit1=G, bit2=B).  When the
 * brightness ramp reaches zero, the mask advances to the next color.
 */
static void effect_breathing(uint8_t buf[][3], bool reset)
{
  static bool dir;
  static uint8_t channel_mask;
  static float brightness;

  if (reset) {
    brightness = 0;
    dir = true;
    channel_mask = 1;
  }

  if (dir) {
    brightness++;
    if (brightness > BREATH_MAX)
      dir = false;
  } else {
    brightness--;
    if (brightness <= 0) {
      dir = true;
      channel_mask++;
      if (channel_mask >= 8)
        channel_mask = 1;
    }
  }

  uint8_t val = (uint8_t)(brightness * BREATH_STEP);
  for (int i = 0; i < NBR_LEDS; i++) {
    buf[i][0] = (channel_mask & 0x01) ? val : 0;
    buf[i][1] = (channel_mask & 0x02) ? val : 0;
    buf[i][2] = (channel_mask & 0x04) ? val : 0;
  }
}

/* --- effect: red spin (clockwise) --- */

static const uint8_t red_ring[NBR_LEDS][3] = {
  { 0, 128, 0 }, { 0, 64, 0 }, { 0, 32, 0 }, { 0, 4, 0 },
  { 0, 2, 0 },   { 0, 1, 0 },  { 0, 0, 0 },  { 0, 0, 0 },
  { 0, 0, 0 },   { 0, 0, 0 },  { 0, 0, 0 },  { 0, 0, 0 },
};

/**
 * @brief  Effect: red gradient spinning clockwise (rotate right by 1 each tick).
 */
static void effect_red_spin(uint8_t buf[][3], bool reset)
{
  if (reset) {
    for (int i = 0; i < NBR_LEDS; i++)
      COPY_RGB(buf[i], red_ring[i]);
  }

  /* rotate right by 1 */
  uint8_t tmp[3];
  COPY_RGB(tmp, buf[0]);
  for (int i = 0; i < NBR_LEDS - 1; i++)
    COPY_RGB(buf[i], buf[i + 1]);
  COPY_RGB(buf[NBR_LEDS - 1], tmp);
}

/* --- effect: color spin (multi-color clockwise) --- */

static const uint8_t color_ring[NBR_LEDS][3] = {
  { 0, 0, 32 },   { 0, 0, 16 }, { 0, 0, 8 }, { 0, 0, 4 },
  { 16, 16, 16 }, { 8, 8, 8 },  { 4, 4, 4 }, { 0, 32, 0 },
  { 0, 16, 0 },   { 0, 8, 0 },  { 0, 4, 0 }, { 0, 2, 0 },
};

/**
 * @brief  Effect: multi-color gradient spinning clockwise.
 */
static void effect_color_spin(uint8_t buf[][3], bool reset)
{
  if (reset) {
    for (int i = 0; i < NBR_LEDS; i++)
      COPY_RGB(buf[i], color_ring[i]);
  }

  uint8_t tmp[3];
  COPY_RGB(tmp, buf[0]);
  for (int i = 0; i < NBR_LEDS - 1; i++)
    COPY_RGB(buf[i], buf[i + 1]);
  COPY_RGB(buf[NBR_LEDS - 1], tmp);
}

/* --- effect: double spin (two colors counter-rotating, merged) --- */

/**
 * @brief  Effect: two color gradients counter-rotating, merged by averaging.
 */
static void effect_double_spin(uint8_t buf[][3], bool reset)
{
  static uint8_t sub1[NBR_LEDS][3];
  static uint8_t sub2[NBR_LEDS][3];

  effect_red_spin(sub1, reset);

  /* counter-clockwise version: rotate left */
  if (reset) {
    for (int i = 0; i < NBR_LEDS; i++)
      COPY_RGB(sub2[i], color_ring[(NBR_LEDS - i) % NBR_LEDS]);
  }
  uint8_t tmp[3];
  COPY_RGB(tmp, sub2[NBR_LEDS - 1]);
  for (int i = NBR_LEDS - 1; i > 0; i--)
    COPY_RGB(sub2[i], sub2[i - 1]);
  COPY_RGB(sub2[0], tmp);

  /* merge by averaging */
  for (int i = 0; i < NBR_LEDS; i++) {
    buf[i][0] = (uint8_t)(((int)sub1[i][0] + (int)sub2[i][0]) / 2);
    buf[i][1] = (uint8_t)(((int)sub1[i][1] + (int)sub2[i][1]) / 2);
    buf[i][2] = (uint8_t)(((int)sub1[i][2] + (int)sub2[i][2]) / 2);
  }
}

/* --- effect function table --- */

static const ledring_effect_fn effect_table[LEDRING_EFFECT_COUNT] = {
  [LEDRING_EFFECT_OFF] = effect_off,
  [LEDRING_EFFECT_COLOR_TEST] = effect_color_test,
  [LEDRING_EFFECT_ATTITUDE] = effect_attitude,
  [LEDRING_EFFECT_GYRO] = effect_gyro,
  [LEDRING_EFFECT_BLINK] = effect_blink,
  [LEDRING_EFFECT_FLASHLIGHT] = effect_flashlight,
  [LEDRING_EFFECT_BREATHING] = effect_breathing,
  [LEDRING_EFFECT_RED_SPIN] = effect_red_spin,
  [LEDRING_EFFECT_COLOR_SPIN] = effect_color_spin,
  [LEDRING_EFFECT_DOUBLE_SPIN] = effect_double_spin,
};

/* --- FreeRTOS software timer callback (50 ms period) --- */

/**
 * @brief  FreeRTOS software timer callback (50 ms period).
 *
 * Checks whether the LED ring module is still active (auto-stops if
 * removed), applies the requested effect change, then pushes the
 * frame buffer to the WS2812 chain.
 */
static void timer_callback(TimerHandle_t timer)
{
  (void)timer;

  /* if module was removed, stop ourselves */
  if (module_manager_get_active() != BSP_MODULE_LED_RING) {
    xTimerStop(timer_handle, 0);
    return;
  }

  static uint8_t led_buf[NBR_LEDS][3];
  bool reset = (requested_effect != current_effect);

  if (reset)
    current_effect = requested_effect;

  if (current_effect < LEDRING_EFFECT_COUNT && effect_table[current_effect])
    effect_table[current_effect](led_buf, reset);

  bsp_ws2812_send(led_buf, NBR_LEDS);
  bsp_ws2812_headlight_set(headlight_on);
}

/* --- public API --- */

void ledring_module_init(void)
{
  if (is_init) {
    /* re-init path: just restart the timer */
    current_effect = LEDRING_EFFECT_OFF;
    requested_effect = LEDRING_EFFECT_OFF;
    headlight_on = false;
    bsp_ws2812_init();
    bsp_ws2812_power_set(true);
    if (timer_handle != NULL)
      xTimerStart(timer_handle, 0);
    return;
  }

  bsp_ws2812_init();
  bsp_ws2812_power_set(true);

  current_effect = LEDRING_EFFECT_OFF;
  requested_effect = LEDRING_EFFECT_OFF;
  headlight_on = false;

  timer_handle =
    xTimerCreate("ledring", pdMS_TO_TICKS(50), pdTRUE, NULL, timer_callback);
  if (timer_handle != NULL)
    xTimerStart(timer_handle, 0);

  is_init = true;
}

void ledring_module_deinit(void)
{
  if (timer_handle != NULL)
    xTimerStop(timer_handle, 0);

  /* turn everything off */
  uint8_t off_buf[NBR_LEDS][3];
  memset(off_buf, 0, sizeof(off_buf));
  bsp_ws2812_send(off_buf, NBR_LEDS);
  bsp_ws2812_headlight_set(false);
  bsp_ws2812_power_set(false);

  current_effect = LEDRING_EFFECT_OFF;
  requested_effect = LEDRING_EFFECT_OFF;
  headlight_on = false;
}

void ledring_module_set_effect(ledring_effect_t effect)
{
  if (effect < LEDRING_EFFECT_COUNT)
    requested_effect = effect;
}

ledring_effect_t ledring_module_get_effect(void)
{
  return current_effect;
}
