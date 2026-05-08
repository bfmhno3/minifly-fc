// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Position and velocity PID controller implementation
 *
 * @details
 * Six PID instances (3 position, 3 velocity).  XY velocity output is
 * scaled to attitude tilt angles; Z velocity output drives raw thrust.
 * Includes auto hover-thrust calibration during stable hover.
 */

#include "control/position_pid.h"

#include <math.h>

#include "services/config_service.h"
#include "comm/commander.h"

/* Thrust output clamp (raw motor command range) */
#define THRUST_MIN 1000
#define THRUST_MAX 60000

/* Velocity PID output limits (cm/s -> attitude deg for XY, raw thrust for Z) */
#define PID_VX_OUT_LIMIT 120.0f
#define PID_VY_OUT_LIMIT 120.0f
#define PID_VZ_OUT_LIMIT 40000

/* Position PID output limits (cm -> cm/s for XY, cm/s for Z) */
#define PID_X_OUT_LIMIT 1200.0f
#define PID_Y_OUT_LIMIT 1200.0f
#define PID_Z_OUT_LIMIT 120.0f

/* Scaling: velocity error (cm/s) -> attitude command (deg) */
#define VEL_XY_SCALE 0.15f
/* Scaling: position error (cm) -> velocity setpoint (cm/s) */
#define POS_XY_SCALE 0.1f

#define THRUST_LPF_ALPHA 0.003f     /* hover thrust LPF coefficient */
#define ACC_Z_STILL_THRESHOLD 35.0f /* |acc_z| below this = still (cm/s^2) */
#define ALT_HOLD_STILL_COUNT 1000 /* ticks of stillness before thrust update */
#define THRUST_BASE_CHANGE_THRESHOLD \
  1000.0f /* min delta to persist new thrust base */

/**
 * @brief State and configuration for a single PID controller instance.
 *
 * @details
 * Holds both tuning parameters (kp/ki/kd) and runtime state (integ,
 * prev_error) in one struct so each axis owns an independent, self-contained
 * controller. Setting out_limit to 0.0f disables output saturation.
 */
typedef struct pid_controller {
  float kp;    // Proportional gain.
  float ki;    // Integral gain.
  float kd;    // Derivative gain.
  float integ; // Accumulated integral term; clamped to [-i_limit, i_limit].
  float prev_error; // Error from the previous update step, used for D term.
  float i_limit;    // Anti-windup clamp on the integral accumulator.
  float out_limit;  // Output saturation limit; 0.0f disables clamping.
  float dt;         // Fixed update interval (s), set once at init.
} pid_controller_t;

static pid_controller_t pid_x;
static pid_controller_t pid_y;
static pid_controller_t pid_z;
static pid_controller_t pid_vx;
static pid_controller_t pid_vy;
static pid_controller_t pid_vz;

static float thrust_lpf;
static uint16_t thrust_base;
static uint16_t althold_count;

/** @brief  Zero-initialise a PID instance with the given time step */
static void pid_init(pid_controller_t *pid, float dt)
{
  pid->kp = 0.0f;
  pid->ki = 0.0f;
  pid->kd = 0.0f;
  pid->integ = 0.0f;
  pid->prev_error = 0.0f;
  pid->i_limit = 500.0f;
  pid->out_limit = 0.0f;
  pid->dt = dt;
}

/** @brief  Clear integral accumulator and previous-error state */
static void pid_reset(pid_controller_t *pid)
{
  pid->integ = 0.0f;
  pid->prev_error = 0.0f;
}

static float pid_update(pid_controller_t *pid, float error)
{
  float output;

  pid->integ += error * pid->dt;
  if (pid->integ > pid->i_limit)
    pid->integ = pid->i_limit;
  else if (pid->integ < -pid->i_limit)
    pid->integ = -pid->i_limit;

  output = pid->kp * error;
  output += pid->ki * pid->integ;
  output += pid->kd * (error - pid->prev_error) / pid->dt;

  if (pid->out_limit != 0.0f) {
    if (output > pid->out_limit)
      output = pid->out_limit;
    else if (output < -pid->out_limit)
      output = -pid->out_limit;
  }

  pid->prev_error = error;
  return output;
}

void position_pid_init(float vel_dt, float pos_dt)
{
  const config_param_t *cfg = config_service_get();

  pid_init(&pid_x, pos_dt);
  pid_init(&pid_y, pos_dt);
  pid_init(&pid_z, pos_dt);
  pid_x.out_limit = PID_X_OUT_LIMIT;
  pid_y.out_limit = PID_Y_OUT_LIMIT;
  pid_z.out_limit = PID_Z_OUT_LIMIT;

  pid_init(&pid_vx, vel_dt);
  pid_init(&pid_vy, vel_dt);
  pid_init(&pid_vz, vel_dt);
  pid_vx.out_limit = PID_VX_OUT_LIMIT;
  pid_vy.out_limit = PID_VY_OUT_LIMIT;
  pid_vz.out_limit = PID_VZ_OUT_LIMIT;

  if (cfg) {
    position_pid_set_gains(&cfg->pid_pos);
    thrust_base = cfg->thrust_base;
  } else {
    thrust_base = 20000;
  }

  thrust_lpf = (float)thrust_base;
  althold_count = 0;
}

void position_pid_reset(void)
{
  pid_reset(&pid_x);
  pid_reset(&pid_y);
  pid_reset(&pid_z);
  pid_reset(&pid_vx);
  pid_reset(&pid_vy);
  pid_reset(&pid_vz);
}

void position_pid_set_gains(const pid_group_pos_t *gains)
{
  pid_x.kp = gains->x.kp;
  pid_x.ki = gains->x.ki;
  pid_x.kd = gains->x.kd;
  pid_y.kp = gains->y.kp;
  pid_y.ki = gains->y.ki;
  pid_y.kd = gains->y.kd;
  pid_z.kp = gains->z.kp;
  pid_z.ki = gains->z.ki;
  pid_z.kd = gains->z.kd;

  pid_vx.kp = gains->vx.kp;
  pid_vx.ki = gains->vx.ki;
  pid_vx.kd = gains->vx.kd;
  pid_vy.kp = gains->vy.kp;
  pid_vy.ki = gains->vy.ki;
  pid_vy.kd = gains->vy.kd;
  pid_vz.kp = gains->vz.kp;
  pid_vz.ki = gains->vz.ki;
  pid_vz.kd = gains->vz.kd;
}

void position_pid_run(setpoint_t *sp, const state_t *state, attitude_t *att_out,
                      float *thrust_out)
{
  /* outer loop: position -> velocity */
  if (sp->mode.x == MODE_ABS)
    sp->velocity.x =
      POS_XY_SCALE * pid_update(&pid_x, sp->position.x - state->position.x);

  if (sp->mode.y == MODE_ABS)
    sp->velocity.y =
      POS_XY_SCALE * pid_update(&pid_y, sp->position.y - state->position.y);

  if (sp->mode.z == MODE_ABS)
    sp->velocity.z = pid_update(&pid_z, sp->position.z - state->position.z);

  /* inner loop: velocity -> attitude / thrust */
  att_out->pitch =
    VEL_XY_SCALE * pid_update(&pid_vx, sp->velocity.x - state->velocity.x);
  att_out->roll =
    VEL_XY_SCALE * pid_update(&pid_vy, sp->velocity.y - state->velocity.y);

  float thrust_raw = pid_update(&pid_vz, sp->velocity.z - state->velocity.z);

  *thrust_out = thrust_raw + (float)thrust_base;
  if (*thrust_out < THRUST_MIN)
    *thrust_out = THRUST_MIN;
  else if (*thrust_out > THRUST_MAX)
    *thrust_out = THRUST_MAX;

  thrust_lpf += (*thrust_out - thrust_lpf) * THRUST_LPF_ALPHA;

  /* Auto-calibrate thrustBase: when hovering steadily for ALT_HOLD_STILL_COUNT
	 * ticks, persist the LPF-smoothed thrust as the new baseline so the drone
	 * adapts to payload or battery changes without manual re-tuning. */
  if (commander_get_key_flight()) {
    if (fabsf(state->acc.z) < ACC_Z_STILL_THRESHOLD) {
      althold_count++;
      if (althold_count > ALT_HOLD_STILL_COUNT) {
        althold_count = 0;
        if (fabsf((float)thrust_base - thrust_lpf) >
            THRUST_BASE_CHANGE_THRESHOLD) {
          config_param_t *cfg = config_service_mut();
          if (cfg) {
            cfg->thrust_base = (uint16_t)thrust_lpf;
            config_service_mark_dirty();
          }
        }
      }
    } else {
      althold_count = 0;
    }
  } else if (!commander_get_key_land()) {
    *thrust_out = 0;
  }
}

float position_pid_get_althold_thrust(void)
{
  return thrust_lpf;
}
