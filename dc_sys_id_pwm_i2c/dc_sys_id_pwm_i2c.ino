#include <Wire.h>
/*Serial Comms config params*/
#define BAUD_RATE 115200
#define ENCODER_I2C_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*Bytes*/

/*Motor Control config params*/
/**PWM config params*/
#define PWM_DIR_PIN 12
#define PWM_OUT_PIN 10
/**BLDC to DC config params*/
#define BLDC_HALL_PIN_1 6
#define BLDC_HALL_PIN_2 5
#define BLDC_HALL_PIN_3 7

/*Test config*/
#define TEST_DURATION 5E6 /*[microseconds]*/
#define PWM_OUT_MAX 255 
#define PWM_OUT_MIN 0 /*0 results in max speed*/
#define PWM_INCREMENT 5

uint16_t request_angle_i2c(int device_address, int num_bytes)
{
  Wire.requestFrom(device_address, num_bytes);
  byte angle_1 = Wire.read();
  byte angle_0 = Wire.read();
  byte temp = 0;
  if (angle_1 & 0xF0) {
    temp = angle_1;
    angle_1 = angle_0;
    angle_0 = temp;
  }
  uint16_t angle_obs = 0;
  angle_obs = ((uint16_t)angle_1) << 8;
  angle_obs += angle_0;
  return angle_obs;
}

void setup() 
{
  /*PWM Setup*/
  pinMode(PWM_OUT_PIN, OUTPUT);
  pinMode(PWM_DIR_PIN, OUTPUT);
  /**Non-prescaled PWM clock: =~ 31.3kHz*/
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  /*Serial Setup*/
  Wire.begin();
  Serial.begin(BAUD_RATE);
  /***Initialize Address Pointer to Angle Register*/  
  Wire.beginTransmission(ENCODER_I2C_ADDRESS);
  Wire.write(ANGLE_REGISTER_ADDRESS);
  Wire.endTransmission();

  /*BLDC to DC Setup*/
  pinMode(BLDC_HALL_PIN_1, OUTPUT);
  pinMode(BLDC_HALL_PIN_2, OUTPUT);
  pinMode(BLDC_HALL_PIN_3, OUTPUT);
  /**Freeze BLDC Commutator to turn into DC driver*/
  digitalWrite(BLDC_HALL_PIN_3, LOW);
  digitalWrite(BLDC_HALL_PIN_2, HIGH);
  digitalWrite(BLDC_HALL_PIN_1, LOW);
}

void loop() 
{
  unsigned long loop_start_time;
  unsigned long loop_duration;
  uint16_t angle_obs;

  delay(3000);
  Serial.println("Starting Test");

  for(int pwm_dir=0; pwm_dir<=1; pwm_dir++)// For each PWM direction
  {
    digitalWrite(PWM_DIR_PIN, pwm_dir);
    for (int pwm_out=PWM_OUT_MIN; pwm_out <=PWM_OUT_MAX; pwm_out+=PWM_INCREMENT)
    {
      analogWrite(PWM_OUT_PIN, pwm_out);
      loop_start_time = micros();
      do
      {
        angle_obs = request_angle_i2c(ENCODER_I2C_ADDRESS, SIZE_OF_ANGLE);
        Serial.print(pwm_dir); Serial.print(" ");
        Serial.print(pwm_out); Serial.print(" ");
        Serial.print(angle_obs); Serial.print(" ");
        Serial.println();
        loop_duration = micros() - loop_start_time;
      } while(loop_duration < TEST_DURATION);      
    }
  }
}

