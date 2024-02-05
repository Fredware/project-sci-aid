#include <Arduino.h>
#include "controller.h"

void controller_initialize(PIConfig *pi_config, PIState *pi_state)
{
    pi_config->kp = 5.0e-5;
    pi_config->ki = 0.0f;
    pi_config->kd = 0.0f; 

    pi_config->tau = 0.003; // derivative LPF time constant; Fc(-3dB) = 50Hz
    pi_config->sampling_period = 0.002; // [seconds] = 1/500 Hz

    pi_state->ctrl_out = 0.0f;

    pi_state->angle_prev = 0.0f;
    pi_state->error_prev = 0.0f;
    pi_state->integral_prev = 0.0f;
    pi_state->derivative_prev = 0.0f;
};

void controller_update(PIConfig *pi_config, PIState *pi_state, uint16_t angle_des, float angle_obs)
{
    float error_obs = angle_des - angle_obs;
    // float time_delta = ((float)(time_obs - pi_state->time_prev)) / 1.0e6;
    // pi_state->error_agg = pi_state->error_agg + (error_obs * time_delta);

    float proportional = pi_config->kp * error_obs;
    float integral = pi_state->integral_prev 
                        + (pi_config->ki * pi_config->sampling_period * 0.5f) * (error_obs - pi_state->error_prev);
    /*TODO: Dynamic Integrator Clamping (Anti-windup)*/
    float derivative = -((2.0f*pi_config->tau - pi_config->sampling_period) * (pi_state->derivative_prev)
                                + (2.0f*pi_config->kd) * (angle_obs - pi_state->angle_prev))
                        /(2.0f*pi_config->tau + pi_config->sampling_period);
    
    pi_state->integral_prev = integral;
    pi_state->derivative_prev = derivative;
    pi_state->angle_prev = angle_obs;
    pi_state->error_prev = error_obs;

    pi_state->ctrl_out = proportional + integral + derivative;
};