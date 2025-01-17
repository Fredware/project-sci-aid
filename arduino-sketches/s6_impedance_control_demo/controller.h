#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
  extern "C"{
#endif

#include <stdint.h>

typedef struct controller_configuration
{
    float k_position;
    float k_torque;
    float k_derivative;

//    float tau; // derivative LPF time constant
    float sampling_period; // [seconds]

    float out_max;
    float out_min;
} ControllerConfig;

typedef struct controller_state
{
    float angle_prev;
    // float error_prev;
    // float integral_prev;
    // float derivative_prev;
    // float time_prev;

    float out;
} ControllerState;

void controller_initialize(ControllerConfig *ctrl_config, ControllerState *ctrl_state);
void controller_update(ControllerConfig *ctrl_config, ControllerState *ctrl_state, float angle_des, float angle_obs, float torque_obs);

#ifdef __cplusplus
  }
#endif

#endif