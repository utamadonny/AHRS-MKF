#include <MPU9250.h>
#define IMU_POLL_DELAY_MS         2
#define MFILTER_SAMPLE_FREQ_HZ    500
#define IMU_LOWPASSFILTER_BANDWIDTH   MPU9250::DLPF_BANDWIDTH_41HZ

#define CALIB_DISABLE
#define ACC_CALIB_DONE
#define MAG_CALIB_DONE

MPU9250 IMU(Wire, 0x68);
Madgwick AHRSFilter;
int status;

typedef union {
  struct {
    float roll;
    float pitch;
    float yaw;
    float Q0,Q1,Q2,Q3;
  };
  uint8_t valArray[sizeof(float) * 3];
} NotifBytesBunch;

NotifBytesBunch AHRSValues;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setDlpfBandwidth(IMU_LOWPASSFILTER_BANDWIDTH);
  
#ifdef ACC_CALIB_DONE
  #ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  IMU.setAccelCalX(0.239477/1.008962);
  IMU.setAccelCalY(0.239477/1.004149);
  IMU.setAccelCalZ(0.112572/0.994379);
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
  IMU.setMagCalX(46.214962/0.936178);
  IMU.setMagCalY(43.776355/1.006211);
  IMU.setMagCalZ(-40.626705/1.066099);
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
  AHRSFilter.begin(MFILTER_SAMPLE_FREQ_HZ);
}
uint8_t loopCount = 0;

void loop() {
  IMU.readSensor(AHRSFilter);
  // if (loopCount++ > 10) {
  //   loopCount = 0;
  //   float rolly = AHRSFilter.getRoll();
  //   if (rolly < 0) {
  //     rolly = 360.0 + rolly;
  //   }
  //   AHRSValues.roll = rolly;
  //   AHRSValues.pitch = AHRSFilter.getPitch();
  //   AHRSValues.yaw = AHRSFilter.getYaw();
  //   AHRSValues.Q0  = AHRSFilter.getq0();
  //   AHRSValues.Q1  = AHRSFilter.getq1();
  //   AHRSValues.Q2  = AHRSFilter.getq2();
  //   AHRSValues.Q3  = AHRSFilter.getq3();

    // Serial.print("R: ");
    // Serial.print(rolly, 2);
    // Serial.print("\tP: ");
    // Serial.print(AHRSValues.pitch , 2);
    // Serial.print("\tY: ");
    // Serial.println(AHRSValues.yaw, 2);
    // Serial.print("\tq1: ");
    // Serial.println(AHRSValues.Q0, 2);
    // Serial.print("\tq2: ");
    // Serial.println(AHRSValues.Q1, 2);
    // Serial.print("\tq3: ");
    // Serial.println(AHRSValues.Q2, 2);
    // Serial.print("\tq4: ");
    // Serial.println(AHRSValues.Q3, 2);
    SerialOut();
  // }
  delay(IMU_POLL_DELAY_MS);
}

void SerialOut(){
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
}