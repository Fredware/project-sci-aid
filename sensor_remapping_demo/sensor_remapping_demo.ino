#include <Wire.h>

#define BAUD_RATE 115200
#define ANGLE_REGISTER_ADDRESS 0x0E
#define ENCODER_ADDRESS 0x36
#define SIZE_OF_ANGLE 2 // [bytes]

#define OBS_MIN 1590
#define OBS_MAX 3260
#define NORM_MIN -835
#define NORM_MAX 835

uint16_t request_angle_i2c(int device_address, int num_bytes){
    Wire.requestFrom(device_address, num_bytes);
    byte angle_1 = Wire.read();
    byte angle_0 = Wire.read();
    byte temp = 0;
    if( angle_1 & 0xF0){
        temp = angle_1;
        angle_1 = angle_0;
        angle_0 = temp;
    }
    uint16_t angle_obs = 0;
    angle_obs = ((uint16_t)angle_1)<<8;
    angle_obs += angle_0;
    return angle_obs;
}

void setup(){
    Serial.begin(BAUD_RATE);

    Wire.begin();
    Wire.beginTransmission(ENCODER_ADDRESS);
    Wire.write(ANGLE_REGISTER_ADDRESS);
    Wire.endTransmission();
}
void loop(){
uint16_t angle_obs = request_angle_i2c(ENCODER_ADDRESS, ANGLE_REGISTER_ADDRESS);
float angle_norm = map(angle_obs, OBS_MIN, OBS_MAX, NORM_MIN, NORM_MAX)/float(NORM_MAX)*100.0;
Serial.print(angle_obs);
Serial.print(" ");
Serial.println(angle_norm, 4);
}