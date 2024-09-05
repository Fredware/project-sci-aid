#define PWM_PIN 10 //OC2A

unsigned int pwm_val = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  /*Non-prescaled PWM: =~ 31.3kHz*/
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1)  | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  Serial.begin(115200);
}

void loop() 
{
  analogWrite(PWM_PIN, pwm_val);
  delay(3000);
  pwm_val = 100;
  // if (pwm_val >= 255) {
  //   pwm_val = 0;
  // }
  Serial.println(pwm_val);
}
