#include "controller.h"

void controller_initialize(ControllerConfig *ctrl_config, ControllerState *ctrl_state){
    ctrl_config->k_position = 0.25f;
    ctrl_config->k_torque = 1.0f;
    ctrl_config->k_derivative = 0.0f;

    ctrl_config->sampling_period = 0.001f;
    ctrl_config->out_max = 255.0f;
    ctrl_config->out_min = -255.0f;

    ctrl_state->angle_prev = 0.0f;
    ctrl_state->out = 0.0f;
}

void controller_update(ControllerConfig *ctrl_config, ControllerState *ctrl_state, float angle_des, float angle_obs, float torque_obs){
    float angle_err = angle_des - angle_obs;
    
    ctrl_state->out = ctrl_config->k_position * angle_err
                    - ctrl_config->k_derivative * (angle_obs - ctrl_state->angle_prev)/ctrl_config->sampling_period
                    + ctrl_config->k_torque * torque_obs;

    if(ctrl_state->out > ctrl_config->out_max){
        ctrl_state->out = ctrl_config->out_max;
    }else if(ctrl_state->out < ctrl_config->out_min){
        ctrl_state->out = ctrl_config->out_min;
    }

    ctrl_state->angle_prev = angle_obs;
}