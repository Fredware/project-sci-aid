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
} PIConfig;

typedef struct controller_state
{
    float angle_prev;
    float error_prev;
    float integral_prev;
    float derivative_prev;
    // float time_prev;

    double ctrl_out;
} PIState;

void controller_initialize(PIConfig *pi_config, PIState *pi_state);
void controller_update(PIConfig *pi_config, PIState *pi_state, float angle_des, float angle_obs);

#ifdef __cplusplus
  }
#endif

#endif