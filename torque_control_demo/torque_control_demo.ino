#include <Wire.h>
#include "controller.h"
#include "setpoint.h"

#define DEBUG_MODE 0
#define LOG_MODE 0

#define LOOP_RATE_PIN 3
#define LOOP_PERIOD 1000 // [mu_sec] = 1kHz

#define BAUD_RATE 115200

#define ADC_PIN A7

#define ANGLE_REGISTER_ADDRESS 0x0E
#define ENCODER_ADDRESS 0x36
#define SIZE_OF_ANGLE 2 // [bytes]

#define PWM_DIR_PIN 12
#define PWM_OUT_PIN 10
#define PWM_MAX 255
#define PWM_MIN 0
#define PWM_DEADBAND 0 

#define ANGLE_OBS_MAX 3232
#define ANGLE_OBS_MIN 1614
#define ANGLE_NORM_MAX 809
#define ANGLE_NORM_MIN -809

#define TORQUE_OBS_MAX 1024
#define TORQUE_OBS_MIN 100
#define TORQUE_NORM_MAX 100
#define TORQUE_NORM_MIN -100
#define TORQUE_BIAS_SAMPLES 4096

/*Controller Globals*/
ControllerConfig ctrl_config;
ControllerState ctrl_state;

/*I2C Sensor*/
uint16_t request_angle_i2c(const int device_address, const int num_bytes)
{
  Wire.requestFrom(device_address, num_bytes);
  byte angle_1 = Wire.read();
  byte angle_0 = Wire.read();
  byte temp = 0;
  
  if (angle_1 & 0xF0){
    temp = angle_1;
    angle_1 = angle_0;
    angle_0 = temp; 
  }
  
  uint16_t angle_obs = 0;
  angle_obs = ((uint16_t)angle_1) << 8;
  angle_obs += angle_0;

  return angle_obs;
}

float torque_bias = 0;


void setup(){
    /*Loop rate debug pin*/
    pinMode(LOOP_RATE_PIN, OUTPUT);
    /*ADC setup*/
    pinMode(ADC_PIN, INPUT);
    /*PWM setup*/
    pinMode(PWM_DIR_PIN, OUTPUT);
    pinMode(PWM_OUT_PIN, OUTPUT);
    /****Non-prescaled phase-correct PWM @~31,3kHz*/
    TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);
    TCCR2B = _BV(CS20);
    // TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0) | _BV(COM1C1) | _BV(COM1C0) | _BV(WGM10);
    // TCCR1B = _BV(CS20);
    /*I2C setup*/
    Wire.begin();
    Serial.begin(BAUD_RATE);
    /****Initialize address pointer to angle register*/
    Wire.beginTransmission(ENCODER_ADDRESS);
    Wire.write(ANGLE_REGISTER_ADDRESS);
    Wire.endTransmission();
    /*Initialize controller configuration*/
    controller_initialize(&ctrl_config, &ctrl_state);
    /*Torque Sensor Calibration*/
    uint16_t torque_obs;
    float torque_norm;
    for(int i =1; i<TORQUE_BIAS_SAMPLES; i++){
      torque_obs = analogRead(ADC_PIN);
      torque_norm = map(torque_obs, TORQUE_OBS_MIN, TORQUE_OBS_MAX, TORQUE_NORM_MIN, TORQUE_NORM_MAX)/float(TORQUE_NORM_MAX)*100.0f;
      torque_bias += torque_norm;
    }
    torque_bias = torque_bias/TORQUE_BIAS_SAMPLES;
}

void loop(){
    /*Broadcast loop start*/
    digitalWrite(LOOP_RATE_PIN, HIGH);
    
    /*Loop rate timekeeping*/
    unsigned long loop_start = micros();
    
    /*Get sensor readings and normalize values*/
    uint16_t angle_obs = request_angle_i2c(ENCODER_ADDRESS, SIZE_OF_ANGLE);
    uint16_t torque_obs = analogRead(ADC_PIN);
    float angle_norm = map(angle_obs, ANGLE_OBS_MIN, ANGLE_OBS_MAX, ANGLE_NORM_MIN, ANGLE_NORM_MAX)/float(ANGLE_NORM_MAX)*100.0f;
    float torque_norm = map(torque_obs, TORQUE_OBS_MIN, TORQUE_OBS_MAX, TORQUE_NORM_MIN, TORQUE_NORM_MAX)/float(TORQUE_NORM_MAX)*100.0f-torque_bias;
    float angle_des = get_setpoint();

    /*Compute control law output*/
    controller_update(&ctrl_config, &ctrl_state, angle_des, angle_norm, torque_norm);

    /*Perform PWM adjustmemts*/
    char pwm_dir = ctrl_state.out > 0;
    float pwm_out = PWM_MAX - fabs(ctrl_state.out);
    if(pwm_out > (PWM_MAX - PWM_DEADBAND)){pwm_out = PWM_MAX;}
    else if(pwm_out < PWM_MIN){pwm_out = PWM_MIN;}
    else{pwm_out = ceil(pwm_out);}
    /*Execute PWM command*/
    if(!DEBUG_MODE)
    {
      digitalWrite(PWM_DIR_PIN, pwm_dir);
      analogWrite(PWM_OUT_PIN, pwm_out);
    }

    /*Broadcast loop stop*/
    digitalWrite(LOOP_RATE_PIN, LOW);
    
    /*Enforce loop rate*/
    long loop_stop = micros() - loop_start;
    while(loop_stop < LOOP_PERIOD){
      // Serial.print(angle_obs, HEX); Serial.print(" ");
      if(LOG_MODE){
        Serial.print(angle_des, 2); Serial.print(" ");
        Serial.print(angle_norm, 2); Serial.print(" ");
        Serial.print(angle_des-angle_norm, 2); Serial.print(" ");
        // Serial.println(torque_obs);
        // Serial.print(torque_norm, 3); Serial.print(" ");
        Serial.print(ctrl_state.out); Serial.print(" ");
        Serial.print(PWM_MAX - fabs(ctrl_state.out),0); Serial.print(" ");
        Serial.print(pwm_out, 0); Serial.print(" ");
        Serial.println();
      }
      loop_stop = micros() - loop_start;
    } 
}