#include "MPU9250.h"
#include <MadgwickAHRS.h> 
#include <MahonyAHRS.h>
// Madgwick filter;
Mahony filter;
unsigned long microsPerReading, microsPrevious;
MPU9250 IMU(Wire,0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68 
int status;
float roll, pitch, yaw;

void setup() {
    // serial to display data
    Serial.begin(115200);
    IMU.begin();
    // filter.begin(25); //comment for mahony
//     IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    // setting the gyroscope full scale range to +/-500 deg/s
//     IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    // setting DLPF bandwidth to 20 Hz
//     IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(39);
microsPerReading = 1000000/25;
microsPrevious = micros();
}

void loop() {
    // read the sensor 
     unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    IMU.readSensor();
    filter.updateIMU(IMU.getGyroX_rads(),IMU.getGyroY_rads(),IMU.getGyroZ_rads(),
    		    IMU.getAccelX_mss(),IMU.getAccelY_mss(),IMU.getAccelZ_mss());
    // filter.update(IMU.getGyroX_rads(),IMU.getGyroY_rads(),IMU.getGyroZ_rads(),
    // 		    IMU.getAccelX_mss(),IMU.getAccelY_mss(),IMU.getAccelZ_mss(),
	// 	        IMU.getMagX_uT(),IMU.getMagY_uT(),IMU.getMagZ_uT());
    roll=filter.getRoll();
    pitch=filter.getPitch();
    yaw=filter.getYaw();
    // display the data 
//     Serial.print(IMU.getAccelX_mss(),4); 
//     Serial.print(" ");
//     Serial.print(IMU.getAccelY_mss()+4,4); 
//     Serial.print(" ");
//     Serial.print(IMU.getAccelZ_mss()-2,4); 
//     Serial.print(" ");
//     Serial.print(IMU.getGyroX_rads(),4); 
//     Serial.print(" ");
//     Serial.print(IMU.getGyroY_rads(),4); 
//     Serial.print(" ");
//     Serial.print(IMU.getGyroZ_rads(),4); 
//     Serial.print(" ");
//     Serial.print(IMU.getMagX_uT(),4); 
//     Serial.print(" ");
//     Serial.print(IMU.getMagY_uT(),4); 
//     Serial.print(" ");
//     Serial.print(IMU.getMagZ_uT(),4); 
//     Serial.print(" ");
//     Serial.print(roll);
//     Serial.print(" ");
//     Serial.print(pitch);
//     Serial.print(" ");
//     Serial.print(yaw);
//     Serial.println(); s
    Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
     // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

