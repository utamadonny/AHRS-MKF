#include "MPU9250.h" 
MPU9250 IMU(Wire,0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68 
int status;


void setup() {
    // serial to display data
    Serial.begin(115200);
    IMU.begin();
}

void loop() {
    // read the sensor 
    IMU.readSensor();
    // display the data 
    Serial.print(IMU.getAccelX_mss(),4); 
    Serial.print(" ");
    Serial.print(IMU.getAccelY_mss(),4); 
    Serial.print(" ");
    Serial.print(IMU.getAccelZ_mss(),4); 
    Serial.print(" ");
    Serial.print(IMU.getGyroX_rads(),4); 
    Serial.print(" ");
    Serial.print(IMU.getGyroY_rads(),4); 
    Serial.print(" ");
    Serial.print(IMU.getGyroZ_rads(),4); 
    Serial.print(" ");
    Serial.print(IMU.getMagX_uT(),4); 
    Serial.print(" ");
    Serial.print(IMU.getMagY_uT(),4); 
    Serial.print(" ");
    Serial.print(IMU.getMagZ_uT(),4); 
    Serial.println(); 
    delay(200);
}
