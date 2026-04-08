// ============================================================
// Code Tested on ARduino Mega with RioRand Brushless Motor Driver 
// Author: Chandra Mauli Dubey
// Date: 03/07/2026
//
// Features:
// - I2C Encoder Feedback 
// - Torque-based assist (KT term)
// - PID Control (P + I + D + Torque + optional accel)
// - 1 kHz control loop
// - Compatible with RioRand PWM + DIR driver
//
// Notes:
// - Torque sensor centered at ~0.63V
// - Output mapped to PWM (0–255)
// ============================================================

#include <Wire.h>
#include <math.h>


// SETTINGS 

// Serial 
#define BAUD_RATE               115200

// I2C Encoder 
// Arduino Mega I2C pins:
// SDA = 20
// SCL = 21
#define ENCODER_ADDRESS         0x36
#define ANGLE_REGISTER_ADDRESS  0x0E
#define SIZE_OF_ANGLE           2

// Angle Scaling 
// Raw encoder counts observed in your mechanism
#define OBS_MIN                 1614.0f
#define OBS_MAX                 3232.0f

// Normalized angle range used in original code
#define NORM_MIN                -809.0f
#define NORM_MAX                809.0f

// Final engineering scaling after normalization
// Original code effectively scales to about -100 to +100
#define ANGLE_SCALE_PERCENT     100.0f

// Setpoint 
// Option 1: fixed setpoint from code
float ANGLE_REF = 40.0f;

// Option 2: setpoint from ADC like whiteboard
bool USE_ADC_SETPOINT = false;
const uint8_t SETPOINT_ADC_PIN = A0;
float SETPOINT_ADC_MIN = 0.0f;
float SETPOINT_ADC_MAX = 1023.0f;
float SETPOINT_OUT_MIN = -100.0f;
float SETPOINT_OUT_MAX = 100.0f;

//  Control Gains 
// Classical PID gains
float KP = 0.015f;
float KI = 0.0f;
float KD = 0.0f;

// Optional extra terms if you want impedance-style extension
float KT = 2.86f;            // torque gain
float KM = 0.0f;               // acceleration / mass-like gain

// Derivative Filter 
// tau = derivative low-pass filter time constant
float DERIV_TAU = 0.008f;

// Loop Timing 
#define LOOP_PERIOD_US         1000UL   // 1 kHz loop
float TS = 0.001f;                      // seconds

// PWM / Motor Pins
// RioRand:
// P   -> Arduino PWM pin
// DIR -> Arduino digital pin
#define PWM_OUT_PIN            10
#define PWM_DIR_PIN            12
#define LOOP_RATE_PIN          3

// RioRand speed input expects standard PWM on P terminal
#define PWM_MAX                255.0f
#define PWM_MIN                0.0f
#define PWM_DBAND              0.0f

// Controller Output Limits 
float CTRL_OUT_MAX = 255.0f;
float CTRL_OUT_MIN = -255.0f;

//  Torque  Sensor 
// Sensor is on A0
// 0.44V is treated as zero signal
// Signal is centered around this point in software, then amplified
bool USE_TORQUE_TERM = true;
const uint8_t TORQUE_SENSOR_PIN = A0;

float ADC_REF_VOLTAGE = 5.0f;
float ADC_MAX_COUNT = 1023.0f;

// Measured resting voltage of your sensor
float TORQUE_SENSOR_ZERO_VOLT = 0.63f;

// Gain after zero-centering
// Increase if response is too small, reduce if it saturates too fast
float TORQUE_SIGNAL_GAIN = 6.0f;

// Small deadband to suppress ADC noise around zero
float TORQUE_DEADBAND_VOLT = 0.0f;

// Final scaling into controller term before KT
float TORQUE_SCALE = 1.0f;

// Optional Acceleration Term
bool USE_ACCEL_TERM = false;

// Optional Angle Filter
bool USE_ANGLE_FILTER = false;

// Display 
bool DISPLAY_CTRL = true;
bool DISPLAY_I2C = false;


// CONTROLLER STATE

float angle_prev = 0.0f;
float error_prev = 0.0f;
float integral_prev = 0.0f;
float derivative_prev = 0.0f;
float velocity_prev = 0.0f;
float ctrl_out = 0.0f;

// Individual control terms for whiteboard-style visibility
float u_p = 0.0f;
float u_i = 0.0f;
float u_d = 0.0f;
float u_t = 0.0f;
float u_m = 0.0f;

// Optional filter memory from original code
float a[3] = {
  0.0016556084568007713f,
  0.0033112169136015426f,
  0.0016556084568007713f
};

float b[3] = {
  1.0f,
  -1.8816490381025f,
  0.8882714719297028f
};

float yn = 0.0f;
float wn = 0.0f;
float wn_1 = 0.0f;
float vn = 0.0f;
float vn_1 = 0.0f;


// HELPER FUNCTIONS


float clampFloat(float x, float xmin, float xmax)
{
  if (x > xmax) return xmax;
  if (x < xmin) return xmin;
  return x;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float filter_signal(float xn)
{
  yn = a[0] * xn + wn_1;
  wn = a[1] * xn - b[1] * yn + vn_1;
  vn = a[2] * xn - b[2] * yn;

  wn_1 = wn;
  vn_1 = vn;

  return yn;
}

float get_setpoint()
{
  if (USE_ADC_SETPOINT) {
    int adc = analogRead(SETPOINT_ADC_PIN);
    return mapFloat((float)adc, SETPOINT_ADC_MIN, SETPOINT_ADC_MAX,
                    SETPOINT_OUT_MIN, SETPOINT_OUT_MAX);
  }

  return ANGLE_REF;
}

uint16_t request_angle_i2c(uint8_t device_address, uint8_t num_bytes)
{
  Wire.requestFrom((int)device_address, (int)num_bytes);

  byte angle_1 = 0;
  byte angle_0 = 0;

  if (Wire.available()) angle_1 = Wire.read();
  if (Wire.available()) angle_0 = Wire.read();

  // Keep same byte-swap behavior as original code
  byte temp = 0;
  if (angle_1 & 0xF0) {
    temp = angle_1;
    angle_1 = angle_0;
    angle_0 = temp;
  }

  uint16_t angle_obs = 0;
  angle_obs = ((uint16_t)angle_1 << 8) | angle_0;

  if (DISPLAY_I2C) {
    Serial.print(angle_1, HEX);
    Serial.print(" ");
    Serial.print(angle_0, HEX);
    Serial.print(" ");
    Serial.println(angle_obs);
  }

  return angle_obs;
}

float read_torque_sensor()
{
  if (!USE_TORQUE_TERM) return 0.0f;

  // Average a few ADC samples for stability
  const int NUM_SAMPLES = 8;
  long adc_sum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    adc_sum += analogRead(TORQUE_SENSOR_PIN);
  }

  float adc_avg = (float)adc_sum / (float)NUM_SAMPLES;

  // Convert ADC count to volts
  float sensor_voltage = (adc_avg * ADC_REF_VOLTAGE) / ADC_MAX_COUNT;

  // Shift so 0.44V becomes zero
  float centered_voltage = sensor_voltage - TORQUE_SENSOR_ZERO_VOLT;

  // Remove tiny noise around zero
  if (fabs(centered_voltage) < TORQUE_DEADBAND_VOLT) {
    centered_voltage = 0.0f;
  }

  // Apply gain so the small signal becomes more useful
  float amplified_signal = centered_voltage * TORQUE_SIGNAL_GAIN;

  // Final controller-scaled value
  return amplified_signal * TORQUE_SCALE;
}


// CONTROLLER INIT

void controller_initialize()
{
  angle_prev = 0.0f;
  error_prev = 0.0f;
  integral_prev = 0.0f;
  derivative_prev = 0.0f;
  velocity_prev = 0.0f;
  ctrl_out = 0.0f;

  u_p = 0.0f;
  u_i = 0.0f;
  u_d = 0.0f;
  u_t = 0.0f;
  u_m = 0.0f;
}


// CONTROLLER UPDATE

void controller_update(float angle_des, float angle_obs)
{
  float error_obs = angle_des - angle_obs;

  // P term 
  u_p = KP * error_obs;

  // I term 
  float integral = integral_prev + (KI * TS * 0.5f) * (error_obs + error_prev);

  float integral_max, integral_min;

  if (CTRL_OUT_MAX > u_p) {
    integral_max = CTRL_OUT_MAX - u_p;
  } else {
    integral_max = 0.0f;
  }

  if (CTRL_OUT_MIN < u_p) {
    integral_min = CTRL_OUT_MIN - u_p;
  } else {
    integral_min = 0.0f;
  }

  integral = clampFloat(integral, integral_min, integral_max);
  u_i = integral;

  // D term 
  float derivative =
    -((2.0f * DERIV_TAU - TS) * derivative_prev
    + (2.0f * KD) * (angle_obs - angle_prev))
    / (2.0f * DERIV_TAU + TS);

  u_d = derivative;

  // Optional torque term 
  u_t = 0.0f;
  if (USE_TORQUE_TERM) {
    float torque_obs = read_torque_sensor();
    u_t = KT * torque_obs;
  }

  // Optional acceleration term 
  u_m = 0.0f;
  if (USE_ACCEL_TERM) {
    float velocity = (angle_obs - angle_prev) / TS;
    float acceleration = (velocity - velocity_prev) / TS;
    u_m = KM * acceleration;
    velocity_prev = velocity;
  }

  // Final control output 
  ctrl_out = u_p + u_i - u_d + u_t + u_m;
  ctrl_out = clampFloat(ctrl_out, CTRL_OUT_MIN, CTRL_OUT_MAX);

  integral_prev = integral;
  derivative_prev = derivative;
  angle_prev = angle_obs;
  error_prev = error_obs;
}


// SETUP

void setup()
{
  pinMode(LOOP_RATE_PIN, OUTPUT);
  pinMode(PWM_DIR_PIN, OUTPUT);
  pinMode(PWM_OUT_PIN, OUTPUT);
  pinMode(SETPOINT_ADC_PIN, INPUT);
  pinMode(TORQUE_SENSOR_PIN, INPUT);

  // Keep same PWM setup on pin 10
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  Wire.begin();
  Serial.begin(BAUD_RATE);

  Wire.beginTransmission(ENCODER_ADDRESS);
  Wire.write(ANGLE_REGISTER_ADDRESS);
  Wire.endTransmission();

  controller_initialize();
}


// LOOP
void loop()
{
  digitalWrite(LOOP_RATE_PIN, HIGH);

  unsigned long start_us = micros();

  // Read angle from I2C encoder 
  uint16_t angle_raw = request_angle_i2c(ENCODER_ADDRESS, SIZE_OF_ANGLE);

  float angle_norm = mapFloat(angle_raw, OBS_MIN, OBS_MAX, NORM_MIN, NORM_MAX);
  angle_norm = (angle_norm / NORM_MAX) * ANGLE_SCALE_PERCENT;

  float angle_obs_flt = angle_norm;
  if (USE_ANGLE_FILTER) {
    angle_obs_flt = filter_signal(angle_norm);
  }

  float angle_des = get_setpoint();

  // Update controller 
  controller_update(angle_des, angle_obs_flt);

  //  RioRand output 
  bool pwm_dir = (ctrl_out < 0.0f);

  // RioRand wants PWM proportional to speed command
  float pwm_out = fabs(ctrl_out);

  if (pwm_out > PWM_MAX) {
    pwm_out = PWM_MAX;
  } else if (pwm_out < PWM_DBAND) {
    pwm_out = 0.0f;
  }

  // Apply output
  digitalWrite(PWM_DIR_PIN, pwm_dir);
  analogWrite(PWM_OUT_PIN, (int)roundf(pwm_out));

  digitalWrite(LOOP_RATE_PIN, LOW);

  if (DISPLAY_CTRL) {
    float adc_debug = analogRead(TORQUE_SENSOR_PIN);
    float voltage_debug = (adc_debug * ADC_REF_VOLTAGE) / ADC_MAX_COUNT;
    float centered_debug = voltage_debug - TORQUE_SENSOR_ZERO_VOLT;
    float amplified_debug = (fabs(centered_debug) < TORQUE_DEADBAND_VOLT) ? 0.0f : centered_debug * TORQUE_SIGNAL_GAIN;

    Serial.print("ref: ");
    Serial.print(angle_des);

    Serial.print("  obs: ");
    Serial.print(angle_obs_flt);

    Serial.print("  err: ");
    Serial.print(error_prev);

    Serial.print("  up: ");
    Serial.print(u_p);

    Serial.print("  ui: ");
    Serial.print(u_i);

    Serial.print("  ud: ");
    Serial.print(u_d);

    Serial.print("  ut: ");
    Serial.print(u_t);

    Serial.print("  um: ");
    Serial.print(u_m);

    Serial.print("  u: ");
    Serial.print(ctrl_out);

    Serial.print("  adcV: ");
    Serial.print(voltage_debug, 3);

    Serial.print("  centeredV: ");
    Serial.print(centered_debug, 3);

    Serial.print("  amp: ");
    Serial.print(amplified_debug, 3);

    Serial.print("  pwm: ");
    Serial.print((int)roundf(pwm_out));

    Serial.print("  dir: ");
    Serial.println(pwm_dir);
  }

  unsigned long elapsed_us = micros() - start_us;
  if (elapsed_us < LOOP_PERIOD_US) {
    delayMicroseconds(LOOP_PERIOD_US - elapsed_us);
  }
}
