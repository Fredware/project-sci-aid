#define ADC_PIN A7
#define BAUD_RATE 115200

void setup(){
pinMode(ADC_PIN, INPUT);
Serial.begin(BAUD_RATE);
}

void loop(){
int flux_density = analogRead(ADC_PIN);
Serial.print(0);
Serial.print(" ");
Serial.print(125);
Serial.print(" ");
Serial.print(flux_density);
Serial.print(" ");
Serial.println(1024);
}