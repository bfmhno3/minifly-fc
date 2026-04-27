#include "comm/commander.h"
#include "services/config_service.h"

#include <math.h>
#include <string.h>
#include "cmsis_os.h"

#define THRUST_MAX        65535.0f
#define LPF_FC            30.0f
#define LPF_T             0.01f
/* alpha = 2*pi*30*0.01 ≈ 1.885, clamped to 1.0 */
#define LPF_ALPHA         1.0f
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

static float lpf_step(float state, float input, float alpha)
{
    return state + alpha * (input - state);
}

static void rotate_carefree(float *roll, float *pitch, float yaw)
{
    float r = *roll;
    float p = *pitch;
    float c = cosf(yaw);
    float s = sinf(yaw);
    *roll  = r * c + p * s;
    *pitch = -r * s + p * c;
}

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

    bool inactive = !cache->active_side;
    cache->buf[inactive] = *val;
    asm volatile("" ::: "memory");
    cache->active_side = inactive;
    cache->timestamp = now;

    g_cmd.last_update_tick = now;
}

void commander_get_setpoint(setpoint_t *sp, const state_t *state)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t dt_ticks = now - g_cmd.last_update_tick;

    /* --- source selection: newest timestamp wins --- */
    ctrl_cache_t *src;
    if (g_cmd.remote_cache.timestamp >= g_cmd.wifi_cache.timestamp)
        src = &g_cmd.remote_cache;
    else
        src = &g_cmd.wifi_cache;

    ctrl_val_t raw = src->buf[src->active_side];

    /* --- watchdog --- */
    bool wdt_stabilize = (dt_ticks > pdMS_TO_TICKS(COMMANDER_WDT_TIMEOUT_STABILIZE));
    bool wdt_shutdown  = (dt_ticks > pdMS_TO_TICKS(COMMANDER_WDT_TIMEOUT_SHUTDOWN));

    if (wdt_shutdown) {
        raw.roll = 0.0f;
        raw.pitch = 0.0f;
        raw.yaw = 0.0f;
        raw.thrust = 0;
        g_cmd.auto_land_active = true;
        g_cmd.auto_land_thrust = g_cmd.lpf_thrust;
    } else if (wdt_stabilize) {
        raw.roll = 0.0f;
        raw.pitch = 0.0f;
        raw.yaw = 0.0f;
    }

    /* --- LPF --- */
    g_cmd.lpf_roll   = lpf_step(g_cmd.lpf_roll,   raw.roll,        LPF_ALPHA);
    g_cmd.lpf_pitch  = lpf_step(g_cmd.lpf_pitch,  raw.pitch,       LPF_ALPHA);
    g_cmd.lpf_yaw    = lpf_step(g_cmd.lpf_yaw,    raw.yaw,         LPF_ALPHA);
    g_cmd.lpf_thrust = lpf_step(g_cmd.lpf_thrust, (float)raw.thrust, LPF_ALPHA);

    memset(sp, 0, sizeof(*sp));

    /* --- emergency stop --- */
    if (g_cmd.bits.emerStop) {
        sp->thrust = 0.0f;
        return;
    }

    /* --- auto-land active --- */
    if (g_cmd.auto_land_active) {
        auto_land_ramp(&g_cmd.auto_land_thrust, &sp->mode);
        sp->velocity.z = -g_cmd.tune.autolandDescent;
        sp->thrust = g_cmd.auto_land_thrust;
        sp->mode.roll = MODE_ABS;
        sp->mode.pitch = MODE_ABS;
        sp->mode.yaw = MODE_VELOCITY;
        return;
    }

    /* --- mode dispatch --- */
    uint8_t ctrl = g_cmd.bits.ctrlMode;

    if (ctrl == 0) {
        /* Manual / rate mode */
        sp->attitudeRate.roll  = g_cmd.lpf_roll  * g_cmd.tune.rateScaleRP;
        sp->attitudeRate.pitch = g_cmd.lpf_pitch * g_cmd.tune.rateScaleRP;
        sp->attitudeRate.yaw   = g_cmd.lpf_yaw   * g_cmd.tune.rateScaleYaw;
        sp->mode.roll  = MODE_VELOCITY;
        sp->mode.pitch = MODE_VELOCITY;
        sp->mode.yaw   = MODE_VELOCITY;
    } else if (ctrl == 1) {
        /* Angle mode */
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
        /* Full-assist (ctrl=3): stub — fall through to angle mode */
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

    /* --- one-key takeoff ramp --- */
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
