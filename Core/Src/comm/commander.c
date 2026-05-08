// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Command interpreter implementation.
 *
 * @details
 * See commander.h for the public API contract.
 */

#include "comm/commander.h"

#include <math.h>
#include <string.h>

#include "cmsis_os.h"

#include "services/config_service.h"

/** @brief Maximum raw thrust value (16-bit). */
#define THRUST_MAX 65535.0f

/** @brief LPF cutoff frequency (Hz) -- used to derive alpha at design time. */
#define LPF_FC 30.0f

/** @brief LPF sample period (s) -- 100 Hz task rate. */
#define LPF_T 0.01f

/**
 * @brief LPF alpha coefficient.
 * alpha = 2*pi*fc*T = 2*pi*30*0.01 ~ 1.885, clamped to 1.0
 * (alpha >= 1.0 means no filtering -- pass-through).
 */
#define LPF_ALPHA 1.0f

/** @brief Degrees to radians conversion factor. */
#define DEG_TO_RAD (M_PI / 180.0f)

/**
 * @brief Runtime command state owned by the commander module.
 *
 * @details
 * This struct centralizes input caches, mode flags, filtered stick values,
 * and one-shot procedure states (auto-land/takeoff) so command interpretation
 * remains deterministic across control-loop ticks.
 */
typedef struct commander {
  commander_bits_t bits; // Packed control/status flags from UI and safety logic.
  ctrl_cache_t remote_cache; // Double-buffered cache for radio controller input.
  ctrl_cache_t wifi_cache; // Double-buffered cache for Wi-Fi controller input.
  yaw_mode_t yaw_mode;     // Active yaw interpretation (x-mode or carefree).
  uint32_t
    last_update_tick; // Last control-input arrival tick for watchdog decisions.

  commander_tune_t tune; // Runtime tuning snapshot loaded from config service.

  float lpf_roll;   // LPF state for roll command (controller units).
  float lpf_pitch;  // LPF state for pitch command (controller units).
  float lpf_yaw;    // LPF state for yaw command (controller units).
  float lpf_thrust; // LPF state for thrust command (raw thrust scale).

  bool auto_land_active;  // True while forced descent procedure is running.
  float auto_land_thrust; // Current thrust during auto-land ramp-down.

  bool takeoff_active;  // True while one-key takeoff ramp is running.
  float takeoff_thrust; // Current thrust during takeoff ramp-up.
} commander_t;

static commander_t g_cmd;

/**
 * @brief  First-order IIR low-pass filter step.
 *
 * Implements: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 */
static float lpf_step(float state, float input, float alpha)
{
  return state + alpha * (input - state);
}

/**
 * @brief  Rotate roll/pitch commands from body frame to world frame.
 *
 * Applies a 2D rotation by -yaw so that "forward" on the stick
 * always means the same heading in carefree mode.
 */
static void rotate_carefree(float *roll, float *pitch, float yaw)
{
  float r = *roll;
  float p = *pitch;
  float c = cosf(yaw);
  float s = sinf(yaw);
  *roll = r * c + p * s;
  *pitch = -r * s + p * c;
}

/**
 * @brief  Ramp thrust downward for auto-land.
 *
 * Decreases thrust by autolandRampStep each call and switches the
 * Z-axis mode to velocity so the position controller commands descent.
 */
static void auto_land_ramp(float *thrust, setpoint_mode_t *mode)
{
  *thrust -= g_cmd.tune.autoland_ramp_step;
  if (*thrust < 0.0f)
    *thrust = 0.0f;
  mode->z = MODE_VELOCITY;
}

void commander_init(void)
{
  memset(&g_cmd, 0, sizeof(g_cmd));
  const config_param_t *cfg = config_service_get();
  if (cfg)
    g_cmd.tune = cfg->cmd_tune;
}

void commander_cache_ctrl_data(ctrl_src_t src, const ctrl_val_t *val)
{
  ctrl_cache_t *cache;
  uint32_t now = xTaskGetTickCount();

  if (src == CTRL_SRC_REMOTER)
    cache = &g_cmd.remote_cache;
  else
    cache = &g_cmd.wifi_cache;

  /* Double-buffer: write to inactive side, then flip. */
  bool inactive = !cache->active_side;
  cache->buf[inactive] = *val;
  /* Memory fence prevents reordering of the store before the flag flip. */
  asm volatile("" ::: "memory");
  cache->active_side = inactive;
  cache->timestamp = now;

  g_cmd.last_update_tick = now;
}

void commander_get_setpoint(setpoint_t *sp, const state_t *state)
{
  uint32_t now = xTaskGetTickCount();
  uint32_t dt_ticks = now - g_cmd.last_update_tick;

  /* --- Source selection: newest timestamp wins --- */
  ctrl_cache_t *src;
  if (g_cmd.remote_cache.timestamp >= g_cmd.wifi_cache.timestamp)
    src = &g_cmd.remote_cache;
  else
    src = &g_cmd.wifi_cache;

  ctrl_val_t raw = src->buf[src->active_side];

  /* --- Watchdog: degrade control on link loss --- */
  bool wdt_stabilize =
    (dt_ticks > pdMS_TO_TICKS(COMMANDER_WDT_TIMEOUT_STABILIZE));
  bool wdt_shutdown =
    (dt_ticks > pdMS_TO_TICKS(COMMANDER_WDT_TIMEOUT_SHUTDOWN));

  if (wdt_shutdown) {
    /* No data for >1s: zero all axes, start auto-land. */
    raw.roll = 0.0f;
    raw.pitch = 0.0f;
    raw.yaw = 0.0f;
    raw.thrust = 0;
    g_cmd.auto_land_active = true;
    g_cmd.auto_land_thrust = g_cmd.lpf_thrust;
  } else if (wdt_stabilize) {
    /* No data for >500ms: hold attitude, keep current thrust. */
    raw.roll = 0.0f;
    raw.pitch = 0.0f;
    raw.yaw = 0.0f;
  }

  /* --- Low-pass filter on all axes --- */
  g_cmd.lpf_roll = lpf_step(g_cmd.lpf_roll, raw.roll, LPF_ALPHA);
  g_cmd.lpf_pitch = lpf_step(g_cmd.lpf_pitch, raw.pitch, LPF_ALPHA);
  g_cmd.lpf_yaw = lpf_step(g_cmd.lpf_yaw, raw.yaw, LPF_ALPHA);
  g_cmd.lpf_thrust = lpf_step(g_cmd.lpf_thrust, (float)raw.thrust, LPF_ALPHA);

  memset(sp, 0, sizeof(*sp));

  /* --- Emergency stop: motors off immediately --- */
  if (g_cmd.bits.emer_stop) {
    sp->thrust = 0.0f;
    return;
  }

  /* --- Auto-land active: controlled descent --- */
  if (g_cmd.auto_land_active) {
    auto_land_ramp(&g_cmd.auto_land_thrust, &sp->mode);
    sp->velocity.z = -g_cmd.tune.autoland_descent;
    sp->thrust = g_cmd.auto_land_thrust;
    sp->mode.roll = MODE_ABS;
    sp->mode.pitch = MODE_ABS;
    sp->mode.yaw = MODE_VELOCITY;
    return;
  }

  /* --- Mode dispatch: rate / angle / full-assist --- */
  uint8_t ctrl = g_cmd.bits.ctrl_mode;

  if (ctrl == 0) {
    /* Manual / rate mode -- angular rate command */
    sp->attitude_rate.roll = g_cmd.lpf_roll * g_cmd.tune.rate_scale_rp;
    sp->attitude_rate.pitch = g_cmd.lpf_pitch * g_cmd.tune.rate_scale_rp;
    sp->attitude_rate.yaw = g_cmd.lpf_yaw * g_cmd.tune.rate_scale_yaw;
    sp->mode.roll = MODE_VELOCITY;
    sp->mode.pitch = MODE_VELOCITY;
    sp->mode.yaw = MODE_VELOCITY;
  } else if (ctrl == 1) {
    /* Angle mode -- absolute angle command */
    float r = g_cmd.lpf_roll * g_cmd.tune.angle_scale_rp;
    float p = g_cmd.lpf_pitch * g_cmd.tune.angle_scale_rp;

    if (g_cmd.yaw_mode == YAW_CAREFREE)
      rotate_carefree(&r, &p, state->attitude.yaw * DEG_TO_RAD);

    sp->attitude.roll = r;
    sp->attitude.pitch = p;
    sp->attitude.yaw = state->attitude.yaw;
    sp->attitude_rate.yaw = g_cmd.lpf_yaw * g_cmd.tune.yaw_rate_scale;
    sp->mode.roll = MODE_ABS;
    sp->mode.pitch = MODE_ABS;
    sp->mode.yaw = MODE_VELOCITY;
  } else {
    /* Full-assist (ctrl=2): same as angle mode for now */
    float r = g_cmd.lpf_roll * g_cmd.tune.angle_scale_rp;
    float p = g_cmd.lpf_pitch * g_cmd.tune.angle_scale_rp;

    if (g_cmd.yaw_mode == YAW_CAREFREE)
      rotate_carefree(&r, &p, state->attitude.yaw * DEG_TO_RAD);

    sp->attitude.roll = r;
    sp->attitude.pitch = p;
    sp->attitude.yaw = state->attitude.yaw;
    sp->attitude_rate.yaw = g_cmd.lpf_yaw * g_cmd.tune.yaw_rate_scale;
    sp->mode.roll = MODE_ABS;
    sp->mode.pitch = MODE_ABS;
    sp->mode.yaw = MODE_VELOCITY;
  }

  /* --- One-key takeoff ramp --- */
  if (g_cmd.takeoff_active) {
    g_cmd.takeoff_thrust += g_cmd.tune.takeoff_ramp_step;
    float target = g_cmd.lpf_thrust;
    if (target < g_cmd.tune.takeoff_min_thrust)
      target = g_cmd.tune.takeoff_min_thrust;
    if (g_cmd.takeoff_thrust >= target) {
      g_cmd.takeoff_thrust = target;
      g_cmd.takeoff_active = false;
    }
    sp->thrust = g_cmd.takeoff_thrust;
  } else {
    sp->thrust = g_cmd.lpf_thrust;
  }
}

uint8_t commander_get_ctrl_mode(void)
{
  return g_cmd.bits.ctrl_mode;
}

bool commander_get_key_flight(void)
{
  return g_cmd.bits.key_flight;
}

bool commander_get_key_land(void)
{
  return g_cmd.bits.key_land;
}

bool commander_get_emer_stop(void)
{
  return g_cmd.bits.emer_stop;
}

void commander_set_ctrl_mode(uint8_t mode)
{
  g_cmd.bits.ctrl_mode = mode & 0x03;
}

void commander_set_key_flight(bool set)
{
  g_cmd.bits.key_flight = set ? 1 : 0;
  if (set) {
    g_cmd.takeoff_active = true;
    g_cmd.takeoff_thrust = 0.0f;
  } else {
    g_cmd.takeoff_active = false;
  }
}

void commander_set_key_land(bool set)
{
  g_cmd.bits.key_land = set ? 1 : 0;
  if (set) {
    g_cmd.auto_land_active = true;
    g_cmd.auto_land_thrust = g_cmd.lpf_thrust;
  } else {
    g_cmd.auto_land_active = false;
  }
}

void commander_set_flight_mode(bool carefree)
{
  g_cmd.bits.flight_mode = carefree ? 1 : 0;
  g_cmd.yaw_mode = carefree ? YAW_CAREFREE : YAW_XMODE;
}

void commander_set_emer_stop(bool set)
{
  g_cmd.bits.emer_stop = set ? 1 : 0;
}
