#include <Wire.h>
/*Serial Comms config params*/
#define BAUD_RATE 115200
#define ENCODER_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*Bytes*/

/*Motor Control config params*/
/**PWM config params*/
#define PWM_PIN 10
#define PWM_MAX 255 
#define PWM_MIN 1 /*0 results in max speed*/
#define DIR_PIN 12
/**BLDC to DC config params*/
#define BLDC_HALL_PIN_1 6
#define BLDC_HALL_PIN_2 5
#define BLDC_HALL_PIN_3 7

const uint16_t ANGLE_SETPOINT = 500; /* = (??) radians*/
const float k_p = 1;


uint16_t request_angle_i2c(int device_address, int num_bytes) {
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
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  /**Non-prescaled PWM clock: =~ 31.3kHz*/
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  /*Serial Setup*/
  Wire.begin();
  Serial.begin(BAUD_RATE);
  /***Initialize Address Pointer to Angle Register*/  
  Wire.beginTransmission(ENCODER_ADDRESS);
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

char fstring[5];
void loop() 
{
  uint16_t angle_obs = request_angle_i2c(ENCODER_ADDRESS, SIZE_OF_ANGLE);
  int angle_error = ANGLE_SETPOINT - angle_obs;
  
  char pwm_dir = angle_error < 0; /*pos -> cw; neg -> ccw*/
  digitalWrite(DIR_PIN, pwm_dir);

  unsigned int pwm_out = ceil(abs(k_p * angle_error));
  if (pwm_out > PWM_MAX){
    pwm_out = PWM_MAX;
  }
  else if(pwm_out < PWM_MIN){
    pwm_out = PWM_MIN;
  }
  analogWrite(PWM_PIN, pwm_out);
  
  Serial.print(ANGLE_SETPOINT); Serial.print(" ");
  sprintf(fstring, "%04d", angle_obs);
  Serial.print(fstring); Serial.print(" ");
  Serial.print(angle_error); Serial.print(" ");
  Serial.print(pwm_out); Serial.print(" ");
  Serial.println();
}
