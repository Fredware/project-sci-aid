#include "controller.h"

void controller_initialize(PIConfig *pi_config, PIState *pi_state)
{
    pi_config->kp = 2e-1;
    pi_config->ki = 0.0f;
    pi_config->kd = 1e-3; 

    pi_config->tau = 0.008; // derivative LPF time constant; Fc(-3dB) = (~20)Hz
    pi_config->sampling_period = 0.001; // [seconds] = 1/500 Hz

    pi_config->ctrl_out_max = 255;
    pi_config->ctrl_out_min = -255;
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
                        + (pi_config->ki * pi_config->sampling_period * 0.5f) * (error_obs + pi_state->error_prev);
    /*Dynamic Integrator Clamping (Anti-windup)*/
    /***Add integral action only if P-term is not already saturating the output*/
    float integral_max, integral_min;
    if(pi_config->ctrl_out_max > proportional){
        integral_max = pi_config->ctrl_out_max - proportional;
    } 
    else{
        integral_max = 0.0f;
    }

    if(pi_config->ctrl_out_min < proportional){
        integral_min = pi_config->ctrl_out_min - proportional;
    }
    else{
        integral_min = 0.0f;
    }

    if(integral > integral_max){
        integral = integral_max;
    }
    else if (integral < integral_min){
        integral = integral_min;
    }   

    float derivative = -((2.0*pi_config->tau - pi_config->sampling_period) * (pi_state->derivative_prev)
                                + (2.0f*pi_config->kd) * (angle_obs - pi_state->angle_prev))
                        /(2.0*pi_config->tau + pi_config->sampling_period);
    
    pi_state->integral_prev = integral;
    pi_state->derivative_prev = derivative;
    pi_state->angle_prev = angle_obs;
    pi_state->error_prev = error_obs;

    pi_state->ctrl_out = proportional + integral + derivative;
    if(pi_state->ctrl_out > pi_config->ctrl_out_max){
        pi_state->ctrl_out = pi_config->ctrl_out_max;
    }
    else if(pi_state->ctrl_out < pi_config->ctrl_out_min){
        pi_state->ctrl_out = pi_config->ctrl_out_min;
    }
};