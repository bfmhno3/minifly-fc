// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Command interpreter implementation.
 *
 * @details
 * See commander.h for the public API contract.
 */

#include "comm/commander.h"
#include "services/config_service.h"

#include <math.h>
#include <string.h>
#include "cmsis_os.h"

/** @brief Maximum raw thrust value (16-bit). */
#define THRUST_MAX        65535.0f

/** @brief LPF cutoff frequency (Hz) -- used to derive alpha at design time. */
#define LPF_FC            30.0f

/** @brief LPF sample period (s) -- 100 Hz task rate. */
#define LPF_T             0.01f

/**
 * @brief LPF alpha coefficient.
 * alpha = 2*pi*fc*T = 2*pi*30*0.01 ~ 1.885, clamped to 1.0
 * (alpha >= 1.0 means no filtering -- pass-through).
 */
#define LPF_ALPHA         1.0f

/** @brief Degrees to radians conversion factor. */
#define DEG_TO_RAD        (M_PI / 180.0f)

struct commander {
    commander_bits_t bits;
    ctrl_cache_t remote_cache;
    ctrl_cache_t wifi_cache;
    yaw_mode_t yaw_mode;
    uint32_t last_update_tick;

    commander_tune_t tune;

    float lpf_roll;
    float lpf_pitch;
    float lpf_yaw;
    float lpf_thrust;

    bool auto_land_active;
    float auto_land_thrust;

    bool takeoff_active;
    float takeoff_thrust;
};

static struct commander g_cmd;

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
    *roll  = r * c + p * s;
    *pitch = -r * s + p * c;
}

/**
 * @brief  Ramp thrust downward for auto-land.
 *
 * Decreases thrust by autolandRampStep each call and switches the
 * Z-axis mode to velocity so the position controller commands descent.
 */
static void auto_land_ramp(float *thrust, mode_t *mode)
{
    *thrust -= g_cmd.tune.autolandRampStep;
    if (*thrust < 0.0f)
        *thrust = 0.0f;
    mode->z = MODE_VELOCITY;
}

void commander_init(void)
{
    memset(&g_cmd, 0, sizeof(g_cmd));
    const config_param_t *cfg = config_service_get();
    if (cfg)
        g_cmd.tune = cfg->cmdTune;
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
    bool wdt_stabilize = (dt_ticks > pdMS_TO_TICKS(COMMANDER_WDT_TIMEOUT_STABILIZE));
    bool wdt_shutdown  = (dt_ticks > pdMS_TO_TICKS(COMMANDER_WDT_TIMEOUT_SHUTDOWN));

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
    g_cmd.lpf_roll   = lpf_step(g_cmd.lpf_roll,   raw.roll,        LPF_ALPHA);
    g_cmd.lpf_pitch  = lpf_step(g_cmd.lpf_pitch,  raw.pitch,       LPF_ALPHA);
    g_cmd.lpf_yaw    = lpf_step(g_cmd.lpf_yaw,    raw.yaw,         LPF_ALPHA);
    g_cmd.lpf_thrust = lpf_step(g_cmd.lpf_thrust, (float)raw.thrust, LPF_ALPHA);

    memset(sp, 0, sizeof(*sp));

    /* --- Emergency stop: motors off immediately --- */
    if (g_cmd.bits.emerStop) {
        sp->thrust = 0.0f;
        return;
    }

    /* --- Auto-land active: controlled descent --- */
    if (g_cmd.auto_land_active) {
        auto_land_ramp(&g_cmd.auto_land_thrust, &sp->mode);
        sp->velocity.z = -g_cmd.tune.autolandDescent;
        sp->thrust = g_cmd.auto_land_thrust;
        sp->mode.roll = MODE_ABS;
        sp->mode.pitch = MODE_ABS;
        sp->mode.yaw = MODE_VELOCITY;
        return;
    }

    /* --- Mode dispatch: rate / angle / full-assist --- */
    uint8_t ctrl = g_cmd.bits.ctrlMode;

    if (ctrl == 0) {
        /* Manual / rate mode -- angular rate command */
        sp->attitudeRate.roll  = g_cmd.lpf_roll  * g_cmd.tune.rateScaleRP;
        sp->attitudeRate.pitch = g_cmd.lpf_pitch * g_cmd.tune.rateScaleRP;
        sp->attitudeRate.yaw   = g_cmd.lpf_yaw   * g_cmd.tune.rateScaleYaw;
        sp->mode.roll  = MODE_VELOCITY;
        sp->mode.pitch = MODE_VELOCITY;
        sp->mode.yaw   = MODE_VELOCITY;
    } else if (ctrl == 1) {
        /* Angle mode -- absolute angle command */
        float r = g_cmd.lpf_roll  * g_cmd.tune.angleScaleRP;
        float p = g_cmd.lpf_pitch * g_cmd.tune.angleScaleRP;

        if (g_cmd.yaw_mode == YAW_CAREFREE)
            rotate_carefree(&r, &p, state->attitude.yaw * DEG_TO_RAD);

        sp->attitude.roll  = r;
        sp->attitude.pitch = p;
        sp->attitude.yaw   = state->attitude.yaw;
        sp->attitudeRate.yaw = g_cmd.lpf_yaw * g_cmd.tune.yawRateScale;
        sp->mode.roll  = MODE_ABS;
        sp->mode.pitch = MODE_ABS;
        sp->mode.yaw   = MODE_VELOCITY;
    } else {
        /* Full-assist (ctrl=2): same as angle mode for now */
        float r = g_cmd.lpf_roll  * g_cmd.tune.angleScaleRP;
        float p = g_cmd.lpf_pitch * g_cmd.tune.angleScaleRP;

        if (g_cmd.yaw_mode == YAW_CAREFREE)
            rotate_carefree(&r, &p, state->attitude.yaw * DEG_TO_RAD);

        sp->attitude.roll  = r;
        sp->attitude.pitch = p;
        sp->attitude.yaw   = state->attitude.yaw;
        sp->attitudeRate.yaw = g_cmd.lpf_yaw * g_cmd.tune.yawRateScale;
        sp->mode.roll  = MODE_ABS;
        sp->mode.pitch = MODE_ABS;
        sp->mode.yaw   = MODE_VELOCITY;
    }

    /* --- One-key takeoff ramp --- */
    if (g_cmd.takeoff_active) {
        g_cmd.takeoff_thrust += g_cmd.tune.takeoffRampStep;
        float target = g_cmd.lpf_thrust;
        if (target < g_cmd.tune.takeoffMinThrust)
            target = g_cmd.tune.takeoffMinThrust;
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
    return g_cmd.bits.ctrlMode;
}

bool commander_get_key_flight(void)
{
    return g_cmd.bits.keyFlight;
}

bool commander_get_key_land(void)
{
    return g_cmd.bits.keyLand;
}

bool commander_get_emer_stop(void)
{
    return g_cmd.bits.emerStop;
}

void commander_set_ctrl_mode(uint8_t mode)
{
    g_cmd.bits.ctrlMode = mode & 0x03;
}

void commander_set_key_flight(bool set)
{
    g_cmd.bits.keyFlight = set ? 1 : 0;
    if (set) {
        g_cmd.takeoff_active = true;
        g_cmd.takeoff_thrust = 0.0f;
    } else {
        g_cmd.takeoff_active = false;
    }
}

void commander_set_key_land(bool set)
{
    g_cmd.bits.keyLand = set ? 1 : 0;
    if (set) {
        g_cmd.auto_land_active = true;
        g_cmd.auto_land_thrust = g_cmd.lpf_thrust;
    } else {
        g_cmd.auto_land_active = false;
    }
}

void commander_set_flight_mode(bool carefree)
{
    g_cmd.bits.flightMode = carefree ? 1 : 0;
    g_cmd.yaw_mode = carefree ? YAW_CAREFREE : YAW_XMODE;
}

void commander_set_emer_stop(bool set)
{
    g_cmd.bits.emerStop = set ? 1 : 0;
}
