#include "controller.h"

void controller_initialize(PIConfig *pi_config, PIState *pi_state)
{
    pi_config->kp = 5.0e-5;
    pi_config->ki = 0;

    pi_state->ctrl_out = 0;
    pi_state->error_agg = 0;
    pi_state->time_prev = 0;
};

void controller_update(PIConfig *pi_config, PIState *pi_state, uint16_t angle_des, float angle_obs, unsigned long time_obs)
{
    float error_obs = angle_des - angle_obs;
    float time_delta = ((float)(time_obs - pi_state->time_prev)) / 1.0e6;
    pi_state->error_agg = pi_state->error_agg + (error_obs * time_delta);

    float proportional = pi_config->kp * error_obs;
    float integral = pi_config->ki * pi_state->error_agg;

    pi_state->ctrl_out = proportional + integral;
};