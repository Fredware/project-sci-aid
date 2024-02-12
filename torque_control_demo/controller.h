#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
  extern "C"{
#endif

#include <stdint.h>

typedef struct controller_configuration
{
    float kp;
    float ki;
    float kd;

    float tau; // derivative LPF time constant
    float sampling_period; // [seconds]

    float ctrl_out_max;
    float ctrl_out_min;
} ControllerConfig;

typedef struct controller_state
{
    float angle_prev;
    float error_prev;
    float integral_prev;
    float derivative_prev;
    // float time_prev;

    float ctrl_out;
} ControllerState;

void controller_initialize(ControllerConfig *ctrl_config, ControllerState *ctrl_state);
void controller_update(ControllerConfig *ctrl_config, ControllerState *ctrl_state, float angle_des, float angle_obs, float torque_obs);

#ifdef __cplusplus
  }
#endif

#endif