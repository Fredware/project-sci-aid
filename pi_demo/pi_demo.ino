#include <Wire.h>

#define BAUD_RATE 115200
#define ENCODER_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*Bytes*/

#define PWM_OUT_PIN 11
#define PWM_MAX 255 /*255*/
#define PWM_MIN 0   /*We don't want full range for some reason*/
#define PWM_DIR_PIN 12

#define DISPLAY_MAIN false
#define DISPLAY_I2C true

const uint16_t ANGLE_REF = 2475; /*Vertical position*/
const float k_p = 0.3;
const float k_i = 0;

int prev_err = 0;
int prev_time = 0;
float cumulative_err = 0;

float p_term = 0;
float i_term = 0;

uint16_t request_angle_i2c() {
  Wire.requestFrom(ENCODER_ADDRESS, SIZE_OF_ANGLE);
  byte angle_1 = Wire.read();
  byte angle_0 = Wire.read();
  uint16_t angle_obs = ((uint16_t)angle_1) << 8;
  angle_obs += angle_0;
  if (DISPLAY_I2C){
  Serial.print(angle_1, HEX);
  Serial.print(" ");
  Serial.print(angle_0, HEX);
  Serial.print(" ");
  Serial.print(angle_obs);
  Serial.println();
  }
  return angle_obs;
}

void setup() {
  /*PWM SETUP*/
  pinMode(PWM_OUT_PIN, OUTPUT);
  pinMode(PWM_DIR_PIN, OUTPUT);
  /*I2C SETUP*/
  Wire.begin();
  Serial.begin(BAUD_RATE);
  /***Initialize Address Pointer to Angle Register*/
  Wire.beginTransmission(ENCODER_ADDRESS);
  Wire.write(ANGLE_REGISTER_ADDRESS);
  Wire.endTransmission();
}

void loop() {
  /*Time calculations*/
  unsigned long current_time = micros();
  float delta_sec = ((float)(current_time - prev_time)) / 1.0e6;
  prev_time = current_time;

  /*Error calculation*/
  uint16_t angle_des = ANGLE_REF; /*Replace with Abby's API*/
  uint16_t angle_obs = request_angle_i2c();
  int angle_err = angle_des - angle_obs;
  cumulative_err = cumulative_err + angle_err * delta_sec;

  /*PID update*/
  p_term = k_p * angle_err;
  i_term = k_i * cumulative_err;
  float ctrl_signal = p_term + i_term;

  /*Map control signal to PWM*/
  char pwm_dir = ctrl_signal > 0; /*pos -> cw; neg -> ccw*/
  float pwm_out = PWM_MAX - fabs(ctrl_signal);
  if (pwm_out > PWM_MAX) {
    pwm_out = PWM_MAX;
  } else if (pwm_out < PWM_MIN) {
    pwm_out = PWM_MIN;
  }

  /*Write PWM command*/
  digitalWrite(PWM_DIR_PIN, pwm_dir);
  analogWrite(PWM_OUT_PIN, pwm_out);

  /*Display Trajectory*/
  if (DISPLAY_MAIN) {
    Serial.print(angle_des);
    Serial.print(" ");
    Serial.print(angle_obs);
    Serial.println();
  }
  delay(10);
}
