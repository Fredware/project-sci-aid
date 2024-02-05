#include <Wire.h>
#include "controller.h"

#define BAUD_RATE 115200
#define ENCODER_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*Bytes*/

#define PWM_OUT_PIN 10 /*TIMER 2*/
#define PWM_MAX 255 /*No speed*/
#define PWM_MIN 0   /*Max speed*/
#define PWM_DIR_PIN 12
#define PWM_DBAND 5e-4

#define DISPLAY_CTRL true
#define DISPLAY_I2C false
#define LOOP_RATE_PIN 3

/*User-defined constants*/
const uint16_t ANGLE_REF = 2468; /*Vertical position approx.*/

/*PID vars*/
PIConfig pi_config;
PIState pi_state;

/*Biquad filter vars: Fs=750Hz, Fc=10Hz*/
float a[] = {
    0.0016556084568007713,
    0.0033112169136015426,
    0.0016556084568007713};
float b[] = {
    1.0,
    -1.8816490381025,
    0.8882714719297028};
float yn;
float wn;
float wn_1;
float vn;
float vn_1;

float filter_signal(float xn)
{
  yn = a[0] * xn + wn_1;
  wn = a[1] * xn - b[1] * yn + vn_1;
  vn = a[2] * xn - b[2] * yn;

  wn_1 = wn;
  vn_1 = vn;

  return yn;
}

uint16_t get_setpoint()
{
  return ANGLE_REF;
}

uint16_t request_angle_i2c()
{
  Wire.requestFrom(ENCODER_ADDRESS, SIZE_OF_ANGLE);
  byte angle_1 = Wire.read();
  byte angle_0 = Wire.read();
  byte temp = 0;
  if (angle_1 & 0b11110000){
    temp = angle_1;
    angle_1 = angle_0;
    angle_0 = temp; 
  } 

  uint16_t angle_obs = 0;
  angle_obs = ((uint16_t)angle_1) << 8;
  angle_obs += angle_0;

  if (DISPLAY_I2C)
  {
    Serial.print(angle_1, HEX);
    Serial.print(" ");
    Serial.print(angle_0, HEX);
    Serial.print(" ");
    Serial.print(angle_obs);
    Serial.println();
  }
  return angle_obs;
}

void setup()
{
  /*Loop rate*/
  pinMode(LOOP_RATE_PIN, OUTPUT);
  /*PWM SETUP*/
  pinMode(PWM_DIR_PIN, OUTPUT);
  pinMode(PWM_OUT_PIN, OUTPUT);
  /***Non-prescaled PWM: =~ 31.3kHz*/
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  /*I2C SETUP*/
  Wire.begin();
  Serial.begin(BAUD_RATE);
  /***Initialize Address Pointer to Angle Register*/
  Wire.beginTransmission(ENCODER_ADDRESS);
  Wire.write(ANGLE_REGISTER_ADDRESS);
  Wire.endTransmission();
  /*PID*/
  controller_initialize(&pi_config, &pi_state);
}

void loop()
{
  /*Mark loop start time*/
  digitalWrite(LOOP_RATE_PIN, HIGH);
  /*Time calculations*/
  unsigned long time_obs = micros();
  uint16_t angle_obs = request_angle_i2c();
  // float angle_obs_flt = filter_signal(angle_obs);
  float angle_obs_flt = angle_obs;
  uint16_t angle_des = get_setpoint(); /*TODO: Replace with Abby's API*/

  controller_update(&pi_config, &pi_state, angle_des, angle_obs_flt);

  /*Map control signal to PWM*/
  char pwm_dir = pi_state.ctrl_out > 0; /*pos -> cw; neg -> ccw*/
  float pwm_out = PWM_MAX - fabs(pi_state.ctrl_out);
  /*Output Clamping*/
  if (pwm_out > (PWM_MAX - PWM_DBAND)){ pwm_out = PWM_MAX;}
  else if (pwm_out < PWM_MIN){pwm_out = PWM_MIN;}

  /*Execute PWM command*/
  digitalWrite(PWM_DIR_PIN, pwm_dir);
  analogWrite(PWM_OUT_PIN, pwm_out);

  /*Display Trajectory*/
  if (DISPLAY_CTRL)
  {
    // Serial.print(angle_des);
    // Serial.print(" ");
    // Serial.print(angle_obs);
    // Serial.print(" ");
    // Serial.print(angle_obs_flt);
    // Serial.print(" ");
    // Serial.print(pi_state.ctrl_out);
    Serial.print(pi_state.error_prev);
    Serial.print(" ");
    Serial.print(pi_state.integral_prev);
    Serial.print(" ");
    Serial.print(pi_state.derivative_prev);
    // Serial.print(" ");
    // Serial.print(filt_signal);
    Serial.println();
  }
  /*Mark loop stop time*/
  digitalWrite(LOOP_RATE_PIN, LOW);
  delayMicroseconds(100);
}