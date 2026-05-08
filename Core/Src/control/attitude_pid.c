// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Implementation of the cascaded attitude PID controllers
 *
 * @details
 * Six independent PID instances: three for the angle (outer) loop and three
 * for the rate (inner) loop.  Integral windup is clamped per-axis; output
 * saturation is optional (out_limit == 0 disables it).
 */

#include "control/attitude_pid.h"

#include <math.h>

/* Integral windup limits (degrees for angle loop, deg/s for rate loop) */
#define ANGLE_I_LIMIT_RP 30.0f /* roll/pitch angle I-limit (deg) */
#define ANGLE_I_LIMIT_YAW \
  180.0f /* yaw angle I-limit (deg), wide for continuous heading */
#define RATE_I_LIMIT_RP 500.0f /* roll/pitch rate I-limit (deg/s) */
#define RATE_I_LIMIT_YAW 50.0f /* yaw rate I-limit (deg/s) */

struct pid_s {
  float kp;
  float ki;
  float kd;
  float integ;
  float prev_error;
  float i_limit;
  float out_limit;
  float dt;
};

static struct pid_s pid_angle_roll;
static struct pid_s pid_angle_pitch;
static struct pid_s pid_angle_yaw;
static struct pid_s pid_rate_roll;
static struct pid_s pid_rate_pitch;
static struct pid_s pid_rate_yaw;

/** @brief  Zero-initialise a PID instance with the given time step */
static void pid_init(struct pid_s *pid, float dt)
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
static void pid_reset(struct pid_s *pid)
{
  pid->integ = 0.0f;
  pid->prev_error = 0.0f;
}

/**
 * @brief  Run one PID update step
 *
 * @param[in] pid    PID instance
 * @param[in] error  Setpoint minus measurement
 * @return  Control output (clamped to out_limit if non-zero)
 */
static float pid_update(struct pid_s *pid, float error)
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

void attitude_pid_init(float rate_dt, float angle_dt)
{
  pid_init(&pid_angle_roll, angle_dt);
  pid_init(&pid_angle_pitch, angle_dt);
  pid_init(&pid_angle_yaw, angle_dt);
  pid_angle_roll.i_limit = ANGLE_I_LIMIT_RP;
  pid_angle_pitch.i_limit = ANGLE_I_LIMIT_RP;
  pid_angle_yaw.i_limit = ANGLE_I_LIMIT_YAW;

  pid_init(&pid_rate_roll, rate_dt);
  pid_init(&pid_rate_pitch, rate_dt);
  pid_init(&pid_rate_yaw, rate_dt);
  pid_rate_roll.i_limit = RATE_I_LIMIT_RP;
  pid_rate_pitch.i_limit = RATE_I_LIMIT_RP;
  pid_rate_yaw.i_limit = RATE_I_LIMIT_YAW;
}

void attitude_pid_reset(void)
{
  pid_reset(&pid_angle_roll);
  pid_reset(&pid_angle_pitch);
  pid_reset(&pid_angle_yaw);
  pid_reset(&pid_rate_roll);
  pid_reset(&pid_rate_pitch);
  pid_reset(&pid_rate_yaw);
}

void attitude_pid_set_angle_gains(const pid_group_t *gains)
{
  pid_angle_roll.kp = gains->roll.kp;
  pid_angle_roll.ki = gains->roll.ki;
  pid_angle_roll.kd = gains->roll.kd;
  pid_angle_pitch.kp = gains->pitch.kp;
  pid_angle_pitch.ki = gains->pitch.ki;
  pid_angle_pitch.kd = gains->pitch.kd;
  pid_angle_yaw.kp = gains->yaw.kp;
  pid_angle_yaw.ki = gains->yaw.ki;
  pid_angle_yaw.kd = gains->yaw.kd;
}

void attitude_pid_set_rate_gains(const pid_group_t *gains)
{
  pid_rate_roll.kp = gains->roll.kp;
  pid_rate_roll.ki = gains->roll.ki;
  pid_rate_roll.kd = gains->roll.kd;
  pid_rate_pitch.kp = gains->pitch.kp;
  pid_rate_pitch.ki = gains->pitch.ki;
  pid_rate_pitch.kd = gains->pitch.kd;
  pid_rate_yaw.kp = gains->yaw.kp;
  pid_rate_yaw.ki = gains->yaw.ki;
  pid_rate_yaw.kd = gains->yaw.kd;
}

void attitude_pid_run_angle(const attitude_t *actual, const attitude_t *desired,
                            Axis3f *desired_rate)
{
  float yaw_error;

  desired_rate->x = pid_update(&pid_angle_roll, desired->roll - actual->roll);
  desired_rate->y =
    pid_update(&pid_angle_pitch, desired->pitch - actual->pitch);

  /* Wrap yaw error to [-180, 180] so the shortest rotation path is chosen */
  yaw_error = desired->yaw - actual->yaw;
  if (yaw_error > 180.0f)
    yaw_error -= 360.0f;
  else if (yaw_error < -180.0f)
    yaw_error += 360.0f;
  desired_rate->z = pid_update(&pid_angle_yaw, yaw_error);
}

void attitude_pid_run_rate(const Axis3f *actual_rate,
                           const Axis3f *desired_rate, Axis3f *control_out)
{
  control_out->x = pid_update(&pid_rate_roll, desired_rate->x - actual_rate->x);
  control_out->y =
    pid_update(&pid_rate_pitch, desired_rate->y - actual_rate->y);
  control_out->z = pid_update(&pid_rate_yaw, desired_rate->z - actual_rate->z);
}
