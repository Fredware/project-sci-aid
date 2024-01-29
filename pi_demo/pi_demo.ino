#include <Wire.h>

#define BAUD_RATE 115200
#define ENCODER_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*Bytes*/

#define PWM_OUT_PIN 10
#define PWM_MAX 255 /*255*/
#define PWM_MIN 0   /*We don't want full range for some reason*/
#define PWM_DIR_PIN 12

#define DISPLAY_CTRL true
#define DISPLAY_I2C false
#define LOOP_RATE_PIN 3

/*User-defined constants*/
const uint16_t ANGLE_REF = 2475; /*Vertical position*/
const float K_P = 0.005;//0.3
const float K_I = 0;

/*PID vars*/
int prev_err = 0;
int time_prev = 0;
float error_agg = 0;
float p_term = 0;
float i_term = 0;

/*Biquad filter vars: Fs=532Hz, Fc=2Hz->20Hz*/
float a[] = {
0.011912749651430805,
0.02382549930286161,
0.011912749651430805
};
float b[] = {
1.0,
-1.6683825426944794,
0.7160335413002028
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
  pinMode(PWM_DIR_PIN, OUTPUT);
  pinMode(PWM_OUT_PIN, OUTPUT);
  /***Non-prescaled PWM: =~ 31.3kHz*/
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1)  | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);
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
  unsigned long time_obs = micros();
  uint16_t angle_obs = request_angle_i2c();
  uint16_t angle_des = get_setpoint(); /*Replace with Abby's API*/

  /*Error calculation*/ 
  int error_obs = angle_des - angle_obs;
  float delta_sec = ((float)(time_obs - time_prev)) / 1.0e6;
  error_agg = error_agg + (error_obs * delta_sec);

  /*PID update*/
  p_term = K_P * error_obs;
  i_term = K_I * error_agg;
  float ctrl_signal = p_term + i_term;
  time_prev = time_obs;

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