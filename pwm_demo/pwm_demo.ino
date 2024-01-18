#define PWM 11

unsigned int pwm_val = 0;

void setup() {
  pinMode(PWM, OUTPUT);
}

void loop() 
{
  analogWrite(PWM, pwm_val);
  delay(1000);
  pwm_val += 10;
  if (pwm_val >= 255) {
    pwm_val = 0;
  }
}
