#define SETPOINT_PIN A2

void setup() {
  pinMode(SETPOINT_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  int pot_val = analogRead(SETPOINT_PIN);
  long pos_setpoint = map(pot_val, 0, 1023, -100, 100);
  Serial.print(pot_val);
  Serial.print(" ");
  Serial.println(pos_setpoint);
}
