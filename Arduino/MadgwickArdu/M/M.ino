#include "MPU9250.h" 
#include <Ewma.h>  

Ewma adcFilter1(0.05); //filter used to smooth sensor data

//#define CALIB_DISABLE
 #define ACC_CALIB_DONE
 #define MAG_CALIB_DONE

MPU9250 IMU(Wire,0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68 
int status;
float ax,ay,az,gx,gy,gz,hx,hy,hz,axf,ayf,azf,gxf,gyf,gzf,hyf,hxf,hzf;

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
    Declare();
    SerialOut();
    SerialFilter();
    delay(200);
}
void Declare(){
    ax=IMU.getAccelX_mss()+0.04;
    ay=IMU.getAccelY_mss()+0.36;
    az=IMU.getAccelZ_mss();
    gx=IMU.getGyroX_rads();
    gy=IMu.getGyroY_rads();
    gz=IMU.getGyroZ_rads();
    hx=IMU.getMagX_uT();
    hy=IMU.getMagY_uT();
    hz=IMU.getMagZ_uT();
    axf = adcFilter1.filter(ax);
    ayf = adcFilter1.filter(ay);
    azf = adcFilter1.filter(az);
}
void SerialOut(){
    Serial.print(ax,6); //+0.055
    Serial.print(",");
    Serial.print(ay,6); //+0.46
    Serial.print(",");
    Serial.print(az,6); //-0.01  
    Serial.print(",");
    Serial.print(gx,6); 
    Serial.print(",");
    Serial.print(gy,6); 
    Serial.print(",");
    Serial.print(gz,6); 
    Serial.print(",");
    Serial.print(hx,6); 
    Serial.print(",");
    Serial.print(hy,6); 
    Serial.print(",");
    Serial.print(hz),6); 
}
void SerialFilter(){
    Serial.print(axf,6); //+0.055
    Serial.print(",");
    Serial.print(ayf,6); //+0.46
    Serial.print(",");
    Serial.print(azf,6); //-0.01  
    Serial.print(",");
    Serial.print(gxf,6); 
    Serial.print(",");
    Serial.print(gyf,6); 
    Serial.print(",");
    Serial.print(gzf,6); 
    Serial.print(",");
    Serial.print(hxf,6); 
    Serial.print(",");
    Serial.print(hyf,6); 
    Serial.print(",");
    Serial.print(hzf,6); 
    Serial.println(); 
}

void CalibrationSetup(){
    #ifdef ACC_CALIB_DONE
  #ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  IMU.setAccelCalX(0.185998,1.001139);
  IMU.setAccelCalY(0.089138,1.001757);
  IMU.setAccelCalZ(0.141277,0.993996);
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
  Serial.print(',');
  Serial.println(IMU.getAccelScaleFactorX(), 6);
  Serial.print(F("Y: "));
  Serial.print(IMU.getAccelBiasY_mss(), 6);
  Serial.print(',');
  Serial.println(IMU.getAccelScaleFactorY(), 6);
  Serial.print(F("Z: "));
  Serial.print(IMU.getAccelBiasZ_mss(), 6);
  Serial.print(',');
  Serial.println(IMU.getAccelScaleFactorZ(), 6);
#endif
#ifdef MAG_CALIB_DONE
 #ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  IMU.setMagCalX(47.140327,0.897625);
  IMU.setMagCalY(33.622200,1.047886);
  IMU.setMagCalZ(-17.663494,1.073368);
 #endif
#else
  Serial.print(F("CALIB MAG -- move in figure 8s until I say stop!!!"));
  delay(500);
  IMU.calibrateMag();
  Serial.println(F(" done!"));

  Serial.print(F("X: "));
  Serial.print(IMU.getMagBiasX_uT(), 6);
  Serial.print(',');
  Serial.println(IMU.getMagScaleFactorX(), 6);
  Serial.print(F("Y: "));
  Serial.print(IMU.getMagBiasY_uT(), 6);
  Serial.print(',');
  Serial.println(IMU.getMagScaleFactorY(), 6);
  Serial.print(F("Z: "));
  Serial.print(IMU.getMagBiasZ_uT(), 6);
  Serial.print(',');
  Serial.println(IMU.getMagScaleFactorZ(), 6);
#endif
}
