#include <Wire.h>
#include "controller.h"
// #include "setpoint.h"

#define PWM_ENABLED true /*Enable write operation for PWM_DIR and PWM_OUT*/
#define BAUD_RATE 115200
#define SERIAL_ENABLED true
#define PRINT_SET true
#define PRINT_POS true
#define PRINT_POS_I2C false
#define PRINT_ERR true
#define PRINT_CTRL_DIR false
#define PRINT_CTRL_OUT true
#define PRINT_CTRL_SAT true
#define PRINT_TORQ false
#define PRINT_TORQ_ADC false

#define LOOP_PERIOD 1000 /* [mu_sec] => 1kHz */
#define LOOP_RATE_PIN 3 /* Attach to oscilloscope to verify loop rate*/

#define SETPOINT_PIN A2 /* Potentiometer for desired position */
#define SETPOINT_MIN -90
#define SETPOINT_MAX 90

#define ANGLE_REGISTER_ADDRESS 0x0E
#define ENCODER_ADDRESS 0x36
#define SIZE_OF_ANGLE_OBS 2 // [bytes]
#define ANGLE_OBS_MAX 3220
#define ANGLE_OBS_MIN 1600
#define ANGLE_NORM_MAX 810
#define ANGLE_NORM_MIN -810

#define TORQUE_ADC_PIN A7
#define TORQUE_OBS_MAX 15
#define TORQUE_OBS_MIN -15
#define TORQUE_NORM_MAX 1
#define TORQUE_NORM_MIN -1
#define TORQUE_BIAS_SAMPLES 4096

#define PWM_DIR_PIN 12
#define PWM_OUT_PIN 10 /*OC2A*/
#define PWM_BRK_PIN 8
#define PWM_OUT_MAX 254 /*ShieldV1.0: 255=NO_SPEED and 254=HIGH_SPEED*/
#define PWM_OUT_MIN 1 /*ShieldV1.0: 0=MAX_SPEED and 1=LOW_SPEED*/
#define PWM_DEADBAND 0


/*Controller Globals*/
ControllerConfig ctrl_config;
ControllerState ctrl_state;

/*ADC Position*/
float get_setpoint()
{
  float pos_setpoint = map(analogRead(SETPOINT_PIN), 0, 1023, SETPOINT_MIN, SETPOINT_MAX)/float(SETPOINT_MAX)*100.0f; // map returns type long
  return pos_setpoint;
}

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
    pinMode(TORQUE_ADC_PIN, INPUT);
    /*Configfure to read setpoint from analog pot*/
    pinMode(SETPOINT_PIN, INPUT);
    /*PWM setup*/
    pinMode(PWM_DIR_PIN, OUTPUT);
    pinMode(PWM_OUT_PIN, OUTPUT);
    pinMode(PWM_BRK_PIN, OUTPUT);
    digitalWrite(PWM_BRK_PIN, LOW); /* HIGH:Break; LOW:Continue*/
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
    // uint16_t torque_obs;
    // float torque_norm;
    // for(int i =1; i<TORQUE_BIAS_SAMPLES; i++){
    //   torque_obs = analogRead(ADC_PIN);
    //   torque_norm = map(torque_obs, TORQUE_OBS_MIN, TORQUE_OBS_MAX, TORQUE_NORM_MIN, TORQUE_NORM_MAX)/float(TORQUE_NORM_MAX)*100.0f;
    //   torque_bias += torque_norm;
    // }
    // torque_bias = torque_bias/TORQUE_BIAS_SAMPLES;
}

void loop(){
    /*Signal loop start to oscilloscope*/
    digitalWrite(LOOP_RATE_PIN, HIGH);

    /*Loop rate timekeeping*/
    unsigned long loop_start = micros();
    
    /*Get sensor readings and normalize values*/
    float angle_des = get_setpoint();
    uint16_t angle_obs = request_angle_i2c(ENCODER_ADDRESS, SIZE_OF_ANGLE_OBS);
    uint16_t hall_obs = analogRead(TORQUE_ADC_PIN);
    double torque_obs = -0.0013*hall_obs*torque_obs + 0.4982*hall_obs -157.92;

    float angle_norm = map(angle_obs, ANGLE_OBS_MIN, ANGLE_OBS_MAX, ANGLE_NORM_MIN, ANGLE_NORM_MAX)/float(ANGLE_NORM_MAX)*100.0f;
    float torque_norm = map(torque_obs, TORQUE_OBS_MIN, TORQUE_OBS_MAX, TORQUE_NORM_MIN, TORQUE_NORM_MAX)/float(TORQUE_NORM_MAX)*100.0f-torque_bias;

    /*Compute control law output*/
    controller_update(&ctrl_config, &ctrl_state, angle_des, angle_norm, torque_norm);

    /*Perform PWM adjustmemts*/
    /** **** PWM V1.0 ***** */
    bool pwm_dir = ctrl_state.out > 0;
    float pwm_out = fabs(ctrl_state.out);
    if(pwm_out > (PWM_OUT_MAX - PWM_DEADBAND)) {pwm_out = PWM_OUT_MAX;}
    else if(pwm_out < PWM_OUT_MIN) {pwm_out = PWM_OUT_MIN;}
    else {pwm_out = ceil(pwm_out);}
    /** ******************* */
    
    /** **** PWM V2.0 ***** */
    // float pwm_out = fabs(ctrl_state.out);
    // if (pwm_out <1){pwm_out=1;};
    /** ******************* */

    /*Execute PWM command*/
    if(PWM_ENABLED)
    {
      digitalWrite(PWM_DIR_PIN, pwm_dir);
      analogWrite(PWM_OUT_PIN, pwm_out);
    }

    /*Signal loop stop to oscilloscope*/
    digitalWrite(LOOP_RATE_PIN, LOW);
    
    /*Enforce loop rate*/
    long loop_stop = micros() - loop_start;
    while(loop_stop < LOOP_PERIOD){
      // Serial.print(angle_obs, HEX); Serial.print(" ");
      if(SERIAL_ENABLED){
        if (PRINT_SET) { Serial.print(angle_des, 2); Serial.print(" ");}
        if (PRINT_POS) { Serial.print(angle_norm, 2); Serial.print(" ");}  
        if (PRINT_POS_I2C) { Serial.print(angle_obs); Serial.print(" ");}      
        if (PRINT_ERR) { Serial.print(angle_des-angle_norm, 2); Serial.print(" ");}
        // Serial.print(ctrl_state.out); Serial.print(" ");
        if (PRINT_CTRL_DIR) { Serial.print(pwm_dir); Serial.print(" ");}
        if (PRINT_CTRL_OUT) { Serial.print(ctrl_state.out, 0); Serial.print(" ");}
        if (PRINT_CTRL_SAT) { Serial.print(pwm_out, 0); Serial.print(" ");}
        if (PRINT_TORQ) { Serial.print(torque_norm, 2); Serial.print(" ");}
        if (PRINT_TORQ_ADC){Serial.print(hall_obs); Serial.print(" ");}
        Serial.println();
      }
      loop_stop = micros() - loop_start;
    }
}