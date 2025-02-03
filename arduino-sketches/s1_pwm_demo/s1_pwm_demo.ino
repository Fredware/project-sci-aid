#define PWM_DIR_PIN 12
#define PWM_OUT_PIN 10 //OC2A
#define PWM_BRK_PIN 8


unsigned int pwm_val = 10;
int pwm_increment = 10;

void setup() {
    /*Non-prescaled PWM: =~ 31.3kHz*/
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1)  | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  Serial.begin(115200);

  pinMode(PWM_DIR_PIN, OUTPUT);
  digitalWrite(PWM_DIR_PIN, HIGH);

  pinMode(PWM_BRK_PIN, OUTPUT);
  digitalWrite(PWM_BRK_PIN, LOW); /*HIGH = BREAK; LOW = CONTINUE*/

  pinMode(PWM_OUT_PIN, OUTPUT);
  analogWrite(PWM_OUT_PIN, 1); /*Ideally 0, but 0 = MAX_SPEED = 255*/
}

void loop() 
{
  // if (pwm_val >= 255) {
  //   pwm_val = 255;
  //   pwm_increment = -10; 
  // }
  // if (pwm_val <= 0){
  //   pwm_val = 0;
  //   pwm_increment = 10;
  // }
  int delay_ms = 3000;
  pwm_val = 150;
  Serial.println(pwm_val);
  analogWrite(PWM_OUT_PIN, pwm_val);
  delay(delay_ms);
  pwm_val = 255;
  Serial.println(pwm_val);
  analogWrite(PWM_OUT_PIN, pwm_val);
  delay(delay_ms);
  // digitalWrite(PWM_DIR_PIN, HIGH);
  // delay(1500);
  // digitalWrite(PWM_DIR_PIN, LOW);
}
