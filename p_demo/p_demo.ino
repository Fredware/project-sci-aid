#include <Wire.h>

#define BAUD_RATE 115200
#define ENCODER_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*Bytes*/

#define PWM_PIN 11
#define PWM_MAX 255 /*255*/
#define PWM_MIN 0
#define DIR_PIN 12

const uint16_t ANGLE_SETPOINT = 2445; /*Vertical position*/
const float k_p = 0.05;

void setup() 
{
  /*PWM SETUP*/
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  /*I2C SETUP*/
  Wire.begin();
  Serial.begin(BAUD_RATE);
  /***Initialize Address Pointer to Angle Register*/  
  Wire.beginTransmission(ENCODER_ADDRESS);
  Wire.write(ANGLE_REGISTER_ADDRESS);
  Wire.endTransmission();
}

void loop() 
{
  Wire.requestFrom(ENCODER_ADDRESS, SIZE_OF_ANGLE);
  byte angle_1 = Wire.read();
  byte angle_0 = Wire.read();
  uint16_t angle_meas = ((uint16_t) angle_1) << 8 | angle_0;

  int angle_error = ANGLE_SETPOINT - angle_meas;
  
  char pwm_dir = angle_error > 0; /*pos -> cw; neg -> ccw*/
  digitalWrite(DIR_PIN, pwm_dir);

  unsigned int pwm_out = PWM_MAX - ceil(abs(k_p * angle_error));
  if (pwm_out > PWM_MAX){
    pwm_out = PWM_MAX;
  }
  else if(pwm_out < PWM_MIN){
    pwm_out = PWM_MIN;
  }
  analogWrite(PWM_PIN, pwm_out);
  delay(100);
}
