#define ADC_PIN A7
#define BAUD_RATE 115200

void setup(){
pinMode(ADC_PIN, INPUT);
Serial.begin(BAUD_RATE);
}

void loop(){
int flux_density = analogRead(ADC_PIN);
int force = 0.0048*flux_density*flux_density - 2.7357*flux_density + 376.7266; // polyfit ndeg=2
Serial.print(flux_density);
Serial.print(" ");
Serial.print(force);
Serial.print(" ");
Serial.println("");
}