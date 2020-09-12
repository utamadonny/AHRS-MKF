#include "MPU9250.h" 

//#define CALIB_DISABLE
 #define ACC_CALIB_DONE
 #define MAG_CALIB_DONE

MPU9250 IMU(Wire,0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68 
int status;

void setup() {
    // serial to display data
    Serial.begin(115200);
    while(!Serial) {
    }
    // start communication with IMU
    status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
        }    
CalibrationSetup();
}
void loop() {
    // read the sensor 
    IMU.readSensor();
    // display the data 
    SerialOut();
    delay(200);
}
void SerialOut(){
    Serial.print(IMU.getAccelX_mss(),6); 
    Serial.print(",");
    Serial.print(IMU.getAccelY_mss(),6); 
    Serial.print(",");
    Serial.print(IMU.getAccelZ_mss(),6); 
    Serial.print(",");
    Serial.print(IMU.getGyroX_rads(),6); 
    Serial.print(",");
    Serial.print(IMU.getGyroY_rads(),6); 
    Serial.print(",");
    Serial.print(IMU.getGyroZ_rads(),6); 
    Serial.print(",");
    Serial.print(IMU.getMagX_uT(),6); 
    Serial.print(",");
    Serial.print(IMU.getMagY_uT(),6); 
    Serial.print(",");
    Serial.print(IMU.getMagZ_uT(),6); 
    Serial.println(); 
}

void CalibrationSetup(){
    #ifdef ACC_CALIB_DONE
  #ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  IMU.setAccelCalX(0.179314,1.002417);
  IMU.setAccelCalY(0.101385,1.002250);
  IMU.setAccelCalZ(0.167875,0.996245);
#endif
#else
  Serial.println(F("********** ACC calib **************"));

  char * dirs[6] = { "X+", "X-", "Y+", "Y-", "Z+", "Z-"};

  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(F("Enter when ready for dir "));
    Serial.print((int)(i + 1));
    Serial.print(' ');
    Serial.print(dirs[i]);
    while (! Serial.available() ) {
      delay(10);
    }

    while (Serial.available()) {
      Serial.read();
      delay(5);
      Serial.print('.');
    }
    Serial.println();
    IMU.calibrateAccel();
  }
  Serial.println(F("Acc calib done"));
  Serial.println(F("Vals: "));
  Serial.print(F("X: "));
  Serial.print(IMU.getAccelBiasX_mss(), 6);
  Serial.print('/');
  Serial.println(IMU.getAccelScaleFactorX(), 6);
  Serial.print(F("Y: "));
  Serial.print(IMU.getAccelBiasY_mss(), 6);
  Serial.print('/');
  Serial.println(IMU.getAccelScaleFactorY(), 6);
  Serial.print(F("Z: "));
  Serial.print(IMU.getAccelBiasZ_mss(), 6);
  Serial.print('/');
  Serial.println(IMU.getAccelScaleFactorZ(), 6);
#endif
#ifdef MAG_CALIB_DONE
 #ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  IMU.setMagCalX(39.858528,1.04263);
  IMU.setMagCalY(11.006292,0.913933);
  IMU.setMagCalZ(-11.607842,1.056283);
 #endif
#else
  Serial.print(F("CALIB MAG -- move in figure 8s until I say stop!!!"));
  delay(500);
  IMU.calibrateMag();
  Serial.println(F(" done!"));

  Serial.print(F("X: "));
  Serial.print(IMU.getMagBiasX_uT(), 6);
  Serial.print('/');
  Serial.println(IMU.getMagScaleFactorX(), 6);
  Serial.print(F("Y: "));
  Serial.print(IMU.getMagBiasY_uT(), 6);
  Serial.print('/');
  Serial.println(IMU.getMagScaleFactorY(), 6);
  Serial.print(F("Z: "));
  Serial.print(IMU.getMagBiasZ_uT(), 6);
  Serial.print('/');
  Serial.println(IMU.getMagScaleFactorZ(), 6);
#endif
}
