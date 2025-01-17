#define PWM_PIN 10  //OC2A
#define PWM_DIR_PIN 12

#define BLDC_HALL_PIN_1 6
#define BLDC_HALL_PIN_2 5
#define BLDC_HALL_PIN_3 7

unsigned int pwm_val = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_DIR_PIN, OUTPUT);
  /*Non-prescaled PWM: =~ 31.3kHz*/
  TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  Serial.begin(115200);
  pinMode(BLDC_HALL_PIN_1, OUTPUT);
  pinMode(BLDC_HALL_PIN_2, OUTPUT);
  pinMode(BLDC_HALL_PIN_3, OUTPUT);
  /*BLDC Commutator Freeze to turn into DC*/
  digitalWrite(BLDC_HALL_PIN_3, LOW);
  digitalWrite(BLDC_HALL_PIN_2, HIGH);
  digitalWrite(BLDC_HALL_PIN_1, LOW);
}

bool pwm_dir = HIGH;

void loop() { 
  analogWrite(PWM_PIN, pwm_val);

  pwm_dir = !pwm_dir;
  digitalWrite(PWM_DIR_PIN, pwm_dir);
  Serial.print(pwm_val);
  Serial.print(" ");
  Serial.println(pwm_dir);
  delay(3000);
  
  pwm_dir = !pwm_dir;
  digitalWrite(PWM_DIR_PIN, pwm_dir);
  Serial.print(pwm_val);
  Serial.print(" ");
  Serial.println(pwm_dir);
  delay(3000);

  pwm_val += 15;
  if (pwm_val >= 255) {
    pwm_val = 0;
  }
}
