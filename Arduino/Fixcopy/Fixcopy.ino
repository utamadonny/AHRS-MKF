#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "MPU9250.h" 
#include <MadgwickAHRS.h>

Madgwick filter;
float rollm,pitchm,yawm;
// unsigned long microsPerReading, microsPrevious;

// float rollm2, pitchm2, yawm2, rollb2,pitchb2,yawb2, rollm3, pitchm3, yawm3, rollb3,pitchb3,yawb3;
// Orient:,359.94,8.81,-2.13,Rot:,0.00,-0.12,-0.06,Linear:,-0.08,-0.04,-0.54,Mag:,10.06,23.37,-0.75,Accl:,1.42,0.32,9.13,Accl:,1.50,0.36,9.68,Calibration: Sys=,0, Gyro=,3, Accel=,0, Mag=,0,0.287925,2.605595,-9.456173,0.000956,-0.000377,0.000180,33.799571,33.293048,8.876404,
MPU9250 IMU(Wire,0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int status;
float ax,ay,az,gx,gy,gz,hx,hy,hz; //axf,ayf,azf,gxf,gyf,gzf,hxf,hyf,hzf;
uint16_t SAMPLERATE_DELAY_MS = 50;

// #define CALIB_DISABLE For MPU
#define ACC_CALIB_DONE
#define MAG_CALIB_DONE
#define TCAADDR 0x70 //Multiplexer I2C Addres

void tcaselect(uint8_t i) { //! select for multiplexer
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
    // serial to display data
    // Serial.begin(115200);
  Serial.begin(115200);
  // Serial.println("Orientation Sensor Test"); Serial.println(" ");
  // delay(200);
  tcaselect(1);
   status = IMU.begin();
  if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1);
      // status = 1;
      }
  // Serial.println("MPU Pass");
  tcaselect(0); // 0 = BNO, 1=MPU
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // Serial.println("BNO pass");
  delay(200);
  // tcaselect(1);
  // while(!Serial) {
  // }
   // start communication with IMU
      delay(200);
CalibrationMPU();
}

void loop(void)
{ 
  tcaselect(0);
  AllBNO();
  CalibrationBNO();
    tcaselect(1);
    IMU.readSensor();
    // display the data 
    AllMPU();
    // SerialFilter();
    // delay(200);
  // Serial.println("--");
  delay(SAMPLERATE_DELAY_MS);
}



// #include <Ewma.h>  

// Ewma adcFilter1(0.1); //* filter used to smooth sensor data
// Ewma adcFilter2(0.1); //? Less smoothing - faster to detect changes, but more prone to noise
// Ewma adcFilter3(0.1); //? More smoothing - less prone to noise, but slower to detect changes
// Ewma adcFilter4(0.1); 
// Ewma adcFilter5(0.1); 
// Ewma adcFilter6(0.1); 
// Ewma adcFilter7(0.1); 
// Ewma adcFilter8(0.1); 
// Ewma adcFilter9(0.1); 

void AllBNO(){
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  int8_t boardTemp = bno.getTemp();
  // Serial.println();
  // Serial.print(F("temperature: "));
  // Serial.println(boardTemp);
  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);
}

void AllMPU(){
    ax=IMU.getAccelX_mss();
    ay=IMU.getAccelY_mss();
    az=IMU.getAccelZ_mss();
    gx=IMU.getGyroX_rads();
    gy=IMU.getGyroY_rads();
    gz=IMU.getGyroZ_rads();
    hx=IMU.getMagX_uT();
    hy=IMU.getMagY_uT();
    hz=IMU.getMagZ_uT();
    // filterm2.updateIMU(gx,gy,gz,ax,ay,az);
    filter.update(gx,gy,gz,ax,ay,az,hx,hy,hz);
    rollm = filter.getRoll();
    pitchm = filter.getPitch();
    yawm = filter.getYaw();
    // rollm3 = filterm3.getRoll();
    // pitchm3 = filterm3.getPitch();
    // yawm3 = filterm3.getYaw();
    // axf = adcFilter1.filter(ax);
    // ayf = adcFilter2.filter(ay);
    // azf = adcFilter3.filter(az);
    // gxf = adcFilter4.filter(gx);
    // gyf = adcFilter5.filter(gy);
    // gzf = adcFilter6.filter(gz);
    // hxf = adcFilter7.filter(hx);
    // hyf = adcFilter8.filter(hy);
    // hzf = adcFilter9.filter(hz);
    Serial.print(ax,2); //+0.055
    Serial.print(",");
    Serial.print(ay,2); //+0.46
    Serial.print(",");
    Serial.print(az,2); //-0.01  
    Serial.print(",");
    Serial.print(gx,2); 
    Serial.print(",");
    Serial.print(gy,2); 
    Serial.print(",");
    Serial.print(gz,2); 
    Serial.print(",");
    Serial.print(hx,2); 
    Serial.print(",");
    Serial.print(hy,2); 
    Serial.print(",");
    Serial.print(hz,2);
    Serial.print(",");
    Serial.print(yawm);
    Serial.print(",");
    Serial.print(pitchm);
    Serial.print(",");
    Serial.print(rollm);
    // Serial.print(",");
    // Serial.print(yawm3);
    // Serial.print(",");
    // Serial.print(pitchm3);
    // Serial.print(",");
    // Serial.print(rollm3);
    // Serial.print(",");
    // Serial.println(); 
    // Serial.print(",");
    Serial.println();
}
// void SerialFilter(){
//     Serial.print(axf,6); //+0.055
//     Serial.print(",");
//     Serial.print(ayf,6); //+0.46
//     Serial.print(",");
//     Serial.print(azf,6); //-0.01  
//     Serial.print(",");
//     Serial.print(gxf,6); 
//     Serial.print(",");
//     Serial.print(gyf,6); 
//     Serial.print(",");
//     Serial.print(gzf,6); 
//     Serial.print(",");
//     Serial.print(hxf,6); 
//     Serial.print(",");
//     Serial.print(hyf,6); 
//     Serial.print(",");
//     Serial.print(hzf,6); 
//     Serial.println(); 
// }
void CalibrationBNO(){
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(",");
  Serial.print(system);
  Serial.print(",");
  // Serial.print(" Gyro=");
  // Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  // Serial.print(" Accel=");
  // Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  // Serial.print(" Mag=");
  // Serial.print(",");
  Serial.print(mag);
  Serial.print(",");
}
void CalibrationMPU(){
    #ifdef ACC_CALIB_DONE
  #ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  tcaselect(1);
  IMU.setAccelCalX( -0.252987,1.045735);
  IMU.setAccelCalY(0.000000,1.000000);
  IMU.setAccelCalZ( 0.126709,0.997062);
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
    tcaselect(1);
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
  tcaselect(1);
  IMU.setMagCalX(51.597270,0.900159);
  IMU.setMagCalY(6.342345,1.004222);
  IMU.setMagCalZ(-6.588032,1.119457);
 #endif
#else
  Serial.print(F("CALIB MAG -- move in figure 8s until I say stop!!!"));
  delay(500);
  tcaselect(1);
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

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    // Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    // Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    // Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    // Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    // Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    // Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }
  // Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.print(",");
}
