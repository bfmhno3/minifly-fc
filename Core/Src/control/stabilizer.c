// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  Main flight control loop orchestrator
 *
 * @details
 * Runs at 500 Hz in a dedicated FreeRTOS task.  Subsystems execute at
 * their own rates via RATE_DO_EXECUTE macros.  The pipeline order is:
 * sensor acquire -> attitude est -> optical flow -> position est ->
 * commander setpoint -> fast adjust -> flip -> anomaly detect ->
 * PID control -> motor output.
 */

#include "control/stabilizer.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control/attitude_estimator.h"
#include "control/position_estimator.h"
#include "control/state_control.h"
#include "control/power_control.h"
#include "control/anomaly_detect.h"
#include "control/flip.h"
#include "comm/commander.h"
#include "services/sensors.h"
#include "modules/optical_flow_module.h"

#define ATTITUDE_EST_RATE RATE_250_HZ
#define ATTITUDE_EST_DT (1.0f / ATTITUDE_EST_RATE)

#define POSITION_EST_RATE RATE_250_HZ
#define POSITION_EST_DT (1.0f / POSITION_EST_RATE)

static bool is_init;

static setpoint_t setpoint;
static sensor_data_t sensor_data;
static state_t state;
static control_t control;

/* fast position adjust */
static uint16_t vel_mode_ticks;
static uint16_t abs_mode_ticks;
static float target_height;
static float baro_prev;
static float baro_vel_lpf;

/**
 * @brief  Two-phase altitude adjustment: velocity tracking then absolute hold
 *
 * Phase 1 (vel_mode_ticks): derives barometer velocity, feeds it as a
 * velocity-mode setpoint so the PID drives toward zero vertical speed.
 * Phase 2 (abs_mode_ticks): switches to absolute position hold at the
 * height captured at the end of phase 1.
 */
static void fast_adjust_pos_z(void)
{
  if (vel_mode_ticks > 0) {
    vel_mode_ticks--;
    estimator_reset_height();

    float baro_vel = (sensor_data.baro.asl - baro_prev) / POSITION_EST_DT;
    baro_prev = sensor_data.baro.asl;
    baro_vel_lpf += (baro_vel - baro_vel_lpf) * 0.35f;

    setpoint.mode.z = MODE_VELOCITY;
    state.velocity.z = baro_vel_lpf;
    setpoint.velocity.z = -1.0f * baro_vel_lpf;

    if (vel_mode_ticks == 0) {
      target_height = position_estimator_get_fused_height();
    }
  } else if (abs_mode_ticks > 0) {
    abs_mode_ticks--;
    estimator_reset_all();
    setpoint.mode.z = MODE_ABS;
    setpoint.position.z = target_height;
  }
}

void stabilizer_init(void)
{
  if (is_init)
    return;

  state_control_init();
  power_control_init();
  anomaly_detect_init();
  flip_init();

  memset(&setpoint, 0, sizeof(setpoint));
  memset(&sensor_data, 0, sizeof(sensor_data));
  memset(&state, 0, sizeof(state));
  memset(&control, 0, sizeof(control));

  vel_mode_ticks = 0;
  abs_mode_ticks = 0;
  target_height = 0.0f;
  baro_prev = 0.0f;
  baro_vel_lpf = 0.0f;

  is_init = true;
}

bool stabilizer_test(void)
{
  bool pass = true;

  pass &= state_control_test();
  pass &= power_control_test();

  return pass;
}

void stabilizer_task(void *arg)
{
  (void)arg;

  uint32_t tick = 0;
  uint32_t last_wake_time = xTaskGetTickCount();

  while (!sensors_are_calibrated()) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MAIN_LOOP_DT));
  }

  for (;;) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MAIN_LOOP_DT));

    /* sensor acquisition, 500hz */
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
      sensors_acquire(&sensor_data, tick);
    }

    /* attitude estimation, 250hz */
    if (RATE_DO_EXECUTE(ATTITUDE_EST_RATE, tick)) {
      attitude_estimator_update(&sensor_data.acc, &sensor_data.gyro,
                                ATTITUDE_EST_DT);
      attitude_estimator_get_attitude(&state.attitude);
    }

    /* optical flow update, 100hz */
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
      optical_flow_module_update(&state, 0.01f);
    }

    /* position estimation, 250hz */
    if (RATE_DO_EXECUTE(POSITION_EST_RATE, tick)) {
      pos_estimator_ext_t flow_ext;
      optical_flow_data_t ofd;

      optical_flow_module_get_data(&ofd);
      flow_ext.valid = ofd.valid;
      flow_ext.pos_sum[0] = ofd.pos_sum[0];
      flow_ext.pos_sum[1] = ofd.pos_sum[1];
      flow_ext.vel_lpf[0] = ofd.vel_lpf[0];
      flow_ext.vel_lpf[1] = ofd.vel_lpf[1];
      flow_ext.laser_range = ofd.laser_range;
      flow_ext.laser_quality = ofd.laser_quality;

      position_estimator_update(&state, &sensor_data, &flow_ext,
                                POSITION_EST_DT);
    }

    /* commander setpoint, 100hz */
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
      commander_get_setpoint(&setpoint, &state);
    }

    /* fast position adjust, 250hz */
    if (RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
      fast_adjust_pos_z();
    }

    /* flip check, 500hz, skip in angle ctrl mode */
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
      if (commander_get_ctrl_mode() != 0x03) {
        flip_check(&setpoint, &control, &state);
      }
    }

    /* anomaly detection, every tick */
    anomaly_detect_check(&sensor_data, &state, &control);

    /* pid control, every tick (internal sub-rates) */
    state_control_run(&control, &sensor_data, &state, &setpoint, tick);

    /* motor output, 500hz */
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
      power_control_run(&control);
    }

    tick++;
  }
}

void stabilizer_get_attitude(attitude_t *out)
{
  out->pitch = -state.attitude.pitch;
  out->roll = state.attitude.roll;
  out->yaw = -state.attitude.yaw;
}

float stabilizer_get_baro(void)
{
  return sensor_data.baro.asl;
}

void stabilizer_get_sensor_data(sensor_data_t *out)
{
  *out = sensor_data;
}

void stabilizer_get_state(axis3f_t *acc, axis3f_t *vel, axis3f_t *pos)
{
  acc->x = state.acc.x;
  acc->y = state.acc.y;
  acc->z = state.acc.z;
  vel->x = state.velocity.x;
  vel->y = state.velocity.y;
  vel->z = state.velocity.z;
  pos->x = state.position.x;
  pos->y = state.position.y;
  pos->z = state.position.z;
}

void stabilizer_set_fast_adjust(uint16_t vel_ticks, uint16_t abs_ticks,
                                float height)
{
  if (vel_ticks != 0 && vel_mode_ticks == 0) {
    baro_prev = sensor_data.baro.asl;
    baro_vel_lpf = 0.0f;
    vel_mode_ticks = vel_ticks;
  }
  if (abs_ticks != 0 && abs_mode_ticks == 0) {
    target_height = height;
    abs_mode_ticks = abs_ticks;
  }
}
