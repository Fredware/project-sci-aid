#define PWM_DIR_PIN 12
#define PWM_OUT_PIN 10 //OC2A
#define PWM_BRK_PIN 8

#define PWM_MAX 255
#define PWM_MIN 0

unsigned int pwm_magnitude = 1;
bool pwm_direction = true;
int pwm_increment = 15;
unsigned int delay_ms = 10e3;

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
}

void loop() 
{
  Serial.print(pwm_direction);
  Serial.print(" ");
  Serial.println(pwm_magnitude);
  digitalWrite(PWM_DIR_PIN, pwm_direction);
  analogWrite(PWM_OUT_PIN, pwm_magnitude); /*Ideally 0, but 0 = MAX_SPEED = 255*/
  delay(delay_ms);

  pwm_direction = !pwm_direction;
  pwm_magnitude += pwm_increment;
  if (pwm_magnitude >= PWM_MAX){
    pwm_magnitude = PWM_MAX-1;
    pwm_increment = -1*pwm_increment;
  }
  if (pwm_magnitude <= PWM_MIN){
    pwm_magnitude = PWM_MIN+1;
    pwm_increment = -1*pwm_increment;
  }
}
