#ifndef CONTROL_POSITION_PID_H
#define CONTROL_POSITION_PID_H

#include "control/flight_types.h"
#include "services/config_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void position_pid_init(float vel_dt, float pos_dt);
void position_pid_reset(void);
void position_pid_set_gains(const pid_group_pos_t *gains);
void position_pid_run(setpoint_t *sp, const state_t *state,
		      attitude_t *att_out, float *thrust_out);
float position_pid_get_althold_thrust(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_POSITION_PID_H */
