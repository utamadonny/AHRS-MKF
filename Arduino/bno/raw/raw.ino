#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MadgwickAHRS.h>
// #include <MahonyAHRS.h>

/* This driver reads raw data from the BNO055

  Connections
  ===========
  Connect SCL to analog 5
  Connect SDA to analog 4
  Connect VDD to 3.3V DC
  Connect GROUND to common ground
  History
  =======
  2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)
// !    Chooose Filter
// Mahony filter;
Madgwick filter;
// ! Choose Filter
float roll, pitch, yaw;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup()
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  // ! Mahony tidak pakai sampling frekuensi
  filter.begin(100);
  // ! Un-comment if you use Madgwick Filter

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);  
  // bno.setAxisRemap(REMAP_CONFIG_P6);
  // bno.setAxisSign(REMAP_SIGN_P6);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  /* Display the floating point data */
  // Serial.print("Orientation: ");
  Serial.print(euler.x()); //yaw
  Serial.print(",");
  Serial.print(euler.y()); //pitch 
  Serial.print(",");
  Serial.print(euler.z()); //roll

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(",");
  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());

  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(",");
  Serial.print(gyr.x());
  Serial.print(",");
  Serial.print(gyr.y());
  Serial.print(",");
  Serial.print(gyr.z());

  imu::Vector<3> hag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print(",");
  Serial.print(hag.x());
  Serial.print(",");
  Serial.print(hag.y());
  Serial.print(",");
  Serial.print(hag.z());
//  // Quaternion data/
//  imu::Quaternion quat = bno.getQuat();
//  Serial.print("qW: ");
//  Serial.print(quat.w(), 4);
//  Serial.print(" qX: ");
//  Serial.print(quat.x(), 4);
//  Serial.print(" qY: ");
//  Serial.print(quat.y(), 4);
//  Serial.print(" qZ: ");
//  Serial.print(quat.z(), 4);
//  Serial.print(" ");
//! Choose 6  DoF or 9 DoF. Mahony only use 6 DoF
  // filter.updateIMU(gyr.x(),gyr.y(),gyr.z(),acc.x(),acc.y(),acc.z());
  // if(bno.isFullyCalibrated()){
  filter.update(gyr.y(),gyr.x(),-gyr.z(),-acc.y(),-acc.x(),acc.z(),-hag.y(),hag.x(),hag.z());
  /* Discussion about NED : 
  https://github.com/kriswiner/MPU9250/issues/345
  https://github.com/kriswiner/MPU9250/issues/418
  https://answers.ros.org/question/339362/nwu-to-enu-conversion-for-a-homemade-imu/
  https://github.com/mavlink/mavros/issues/49
  https://answers.ros.org/question/336814/how-to-change-ned-to-enu/
  https://stackoverflow.com/questions/49790453/enu-ned-frame-conversion-using-quaternions
  https://theory.frydom.org/src/coordinate_systems.html
  */
   /* a,g,h
    x=y, y=x, z=z ==> SWU, pitch = roll, roll=pitch, 90=270 | 180=0/360 
    x=-y y=x, z=z ==> NED with opposite rotation on yaw. pitch = roll, roll = -pitch , yield = 0
    x=-y y=-x z=z ==> NED perfect for yaw but not roll and pitch. pitch = -roll, roll = -pitch , diff=0.5d
    (g,g,-g,-a,-a,a,-h,-h,h) ==> NED more perfect 
    (g,g,-g,-a,-a,a,-h,h,h) ==>  NWD (Wrong)
    (g,g,g)


  */ 
  
//! ---------------------------
  pitch=filter.getRoll(); //? swit h for NED ?
  roll=filter.getPitch();
  yaw=filter.getYaw();
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  // }
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("CALIBRATION: Sys=");
  Serial.print(",");
  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
  Serial.print(",");
  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
  Serial.print(",");
  Serial.print(accel, DEC);
    Serial.print(",");
//  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

