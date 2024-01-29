#include <Wire.h>

#define BAUD_RATE 115200
#define ENCODER_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*Bytes*/

#define PWM_OUT_PIN 11
#define PWM_MAX 255 /*255*/
#define PWM_MIN 0   /*We don't want full range for some reason*/
#define PWM_DIR_PIN 12

#define DISPLAY_CTRL true
#define DISPLAY_I2C false
#define LOOP_RATE_PIN 3

/*User-defined constants*/
const uint16_t ANGLE_REF = 2475; /*Vertical position*/
const float K_P = 0.3;
const float K_I = 0;

/*PID vars*/
int prev_err = 0;
int prev_time = 0;
float cumulative_err = 0;
float p_term = 0;
float i_term = 0;

/*Biquad filter vars: Fs=532Hz, Fc=2Hz*/
float a[] = {
0.0001371901572386312,
0.0002743803144772624,
0.0001371901572386312
};
float b[] = {
1.0,
-1.9665975850995905,
0.9671463457285449
};
float yn;
float wn;
float wn_1;
float vn;
float vn_1;

float filter_signal(float xn) {
  yn = a[0] * xn + wn_1;
  wn = a[1] * xn - b[1] * yn + vn_1;
  vn = a[2] * xn - b[2] * yn;

  wn_1 = wn;
  vn_1 = vn;

  return yn;
}

uint16_t get_setpoint() {
  return ANGLE_REF;
}

uint16_t request_angle_i2c() {
  Wire.requestFrom(ENCODER_ADDRESS, SIZE_OF_ANGLE);
  byte angle_1 = Wire.read();
  byte angle_0 = Wire.read();
  uint16_t angle_obs = ((uint16_t)angle_1) << 8;
  angle_obs += angle_0;

  if (DISPLAY_I2C) {
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
  /*Loop rate*/
  pinMode(LOOP_RATE_PIN, OUTPUT);
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
  digitalWrite(LOOP_RATE_PIN, HIGH);
  /*Time calculations*/
  unsigned long current_time = micros();
  float delta_sec = ((float)(current_time - prev_time)) / 1.0e6;
  prev_time = current_time;

  /*Error calculation*/
  uint16_t angle_des = get_setpoint(); /*Replace with Abby's API*/
  uint16_t angle_obs = request_angle_i2c();
  int angle_err = angle_des - angle_obs;
  cumulative_err = cumulative_err + (angle_err * delta_sec);

  /*PID update*/
  p_term = K_P * angle_err;
  i_term = K_I * cumulative_err;
  float ctrl_signal = p_term + i_term;

  float filt_signal = filter_signal(ctrl_signal);

  /*Map control signal to PWM*/
  char pwm_dir = filt_signal > 0; /*pos -> cw; neg -> ccw*/
  float pwm_out = PWM_MAX - fabs(filt_signal);
  if (pwm_out > PWM_MAX) {
    pwm_out = PWM_MAX;
  } else if (pwm_out < PWM_MIN) {
    pwm_out = PWM_MIN;
  }

  /*Write PWM command*/
  digitalWrite(PWM_DIR_PIN, pwm_dir);
  analogWrite(PWM_OUT_PIN, pwm_out);

  /*Display Trajectory*/
  if (DISPLAY_CTRL) {
    // Serial.print(angle_des);
    // Serial.print(" ");
    // Serial.print(angle_obs);
    // Serial.print(" ");
    Serial.print(ctrl_signal);
    Serial.print(" ");
    Serial.print(filt_signal);
    Serial.println();
  }
  digitalWrite(LOOP_RATE_PIN, LOW);
  delay(1);
}