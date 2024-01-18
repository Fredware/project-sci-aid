#include <Wire.h>

#define BAUD_RATE 115200
#define ENCODER_ADDRESS 0x36
#define ANGLE_REGISTER_ADDRESS 0x0E
#define SIZE_OF_ANGLE 2 /*bytes*/

void setup() 
{
  Wire.begin();
  Serial.begin(BAUD_RATE);
  /*Initialize Address Pointer to Angle Register*/  
  Wire.beginTransmission(ENCODER_ADDRESS);
  Wire.write(ANGLE_REGISTER_ADDRESS);
  Wire.endTransmission();
}

void loop() 
{
  Wire.requestFrom(ENCODER_ADDRESS, SIZE_OF_ANGLE);
  byte angle_1 = Wire.read();
  byte angle_0 = Wire.read();
  uint16_t angle_meas = ((uint16_t) angle_1) << 8 | angle_0;
  Serial.print(angle_1, HEX);
  Serial.print(" ");
  Serial.print(angle_0, HEX);
  Serial.print("  ");
  Serial.println(angle_meas);
  delay(500);
}
