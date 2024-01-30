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
} PIConfig;

typedef struct controller_state
{
    float error_agg;
    float time_prev;

    double ctrl_out;
} PIState;

void controller_initialize(PIConfig *pi_config, PIState *pi_state);
void controller_update(PIConfig *pi_config, PIState *pi_state, uint16_t angle_des, float angle_obs, unsigned long time_obs);

#ifdef __cplusplus
  }
#endif

#endif