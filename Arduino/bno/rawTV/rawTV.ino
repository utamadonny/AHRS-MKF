#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MadgwickAHRS.h>
// #include <MahonyAHRS.h>

/* This driver reads raw data from the BNO055 and use filter madgwick or mahony

  Connections
  ===========
  Connect SCL to analog 5
  Connect SDA to analog 4
  Connect VDD to 3.3V DC
  Connect GROUND to common ground
  
  Possible vector values can be:
  - VECTOR_ACCELEROMETER - m/s^2
  - VECTOR_MAGNETOMETER  - uT
  - VECTOR_GYROSCOPE     - deg/s
  - VECTOR_EULER         - degrees
  - VECTOR_LINEARACCEL   - m/s^2
  - VECTOR_GRAVITY       - m/s^2
  
  Discussion about NED : 
  https://github.com/kriswiner/MPU9250/issues/345
  https://github.com/kriswiner/MPU9250/issues/418
  https://answers.ros.org/question/339362/nwu-to-enu-conversion-for-a-homemade-imu/
  https://github.com/mavlink/mavros/issues/49
  https://answers.ros.org/question/336814/how-to-change-ned-to-enu/
  https://stackoverflow.com/questions/49790453/enu-ned-frame-conversion-using-quaternions
  https://theory.frydom.org/src/coordinate_systems.html
  
  MadgwickAHRS data order : 
  used in this code "filter.update([Data Order]);"
  a,g,h
    x=y, y=x, z=z ==> SWU, pitch = roll, roll=pitch, 90=270 | 180=0/360 
    x=-y y=x, z=z ==> NED with opposite rotation on yaw. pitch = roll, roll = -pitch , yield = 0
    x=-y y=-x z=z ==> NED perfect for yaw but not roll and pitch. pitch = -roll, roll = -pitch , diff=0.5d
  (y,x,z,y,x,z,y,x,z) == format below
    (g,g,-g,-a,-a,a,-h,-h,h) ==> NED more perfect diff=0.3 on pitch and roll
    (g,g,-g,-a,-a,a,-h,h,h) ==>  NWD (Wrong)
    (g,g,-g,-a,-a,a,-h,-h,-h) ==> NED , diff=5d on yawc 
    (g,g,-g,-a,-a,a,h,h,h)==> wrong orientation
    (g,g,g,-a,-a,a,-h,-h,h) ==> NED , diff=0.5d on pitch and 0.2d on roll 
    (-g,-g,-g,-a,-a,a,-h,-h,h) ==> NED , diff=1d on pitch, 0.25 on roll
    (g,-g,-g,-a,-a,a,-h,-h,-h) ==> NED , diff=0.3 on pitch, 0.6 on roll

  History
  =======
  2015/MAR/03  - First release (KTOWN)
  2021/AUG/18  - Added Madgwick filter(utamadonny)
*/

/* Set the delay between fresh samples */
#define HZ (100)
#define BetaNN (1)
#define BNO055_SAMPLERATE_DELAY_MS (1000/HZ) // Periode = 10ms = 0.01s --> SampleRate = 1/0.01 = 100Hz 
// #define Rad_to_degrees (57.2957795)
//#define BNO055_SAMPLERATE_DELAY_MS (1000/512)
//int BNO055_SAMPLERATE_DELAY_MS = 1000/100;
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
//  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  // ! Mahony not use frequency samplerate
  filter.begin(HZ); //  ! Un-comment if you use Madgwick Filter. in Hz unit
  filter.setBeta(BetaNN); // ! Un-comment if you use Madgwick Filter

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
//  Serial.print("Current Temperature: ");
//  Serial.print(temp);
//  Serial.println(" C");
//  Serial.println("");

  bno.setExtCrystalUse(true);  
  // bno.setAxisRemap(REMAP_CONFIG_P6);
  // bno.setAxisSign(REMAP_SIGN_P6);
//  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> hag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  // filter.updateIMU(gyr.x(),gyr.y(),gyr.z(),acc.x(),acc.y(),acc.z());
  filter.update(gyr.y(),gyr.x(),-gyr.z(),-acc.y(),-acc.x(),acc.z(),-hag.y(),-hag.x(),hag.z());
  roll=-1*filter.getRoll(); //? swit h for NED ?
  pitch=-1*filter.getPitch();
  yaw=filter.getYaw();
  /* Display orientaion data from filter */
  float ax,ay,az,gx,gy,gz,hx,hy,hz,ro,pi,ya,roa,pia,yaa;
  uint8_t cal_sys,cal_acc,cal_gyr,cal_mag;

  ax = acc.x(); ay = acc.y(); az = acc.z(); gx = gyr.x(); gy = gyr.y(); gz = gyr.z(); hx = hag.x(); hy = hag.y(); hz = hag.z();
  ro = euler.x(); pi = euler.y(); ya = euler.z(); roa=roll; pia=pitch; yaa=yaw; cal_sys=system; cal_acc=accel; cal_gyr=gyro; cal_mag=mag;

  char ax_text[30], ay_text[30], az_text[30], gx_text[30], gy_text[30], gz_text[30], hx_text[30], hy_text[30], hz_text[30], ro_text[30], pi_text[30], ya_text[30], roa_text[30], pia_text[30], yaa_text[30], cal_sys_text[30], cal_acc_text[30], cal_gyr_text[30], cal_mag_text[30];
  dtostrf(ax,10,10,ax_text);
  dtostrf(ay,10,10,ay_text);
  dtostrf(az,10,10,az_text);
  dtostrf(gx,10,10,gx_text);
  dtostrf(gy,10,10,gy_text);
  dtostrf(gz,10,10,gz_text);
  dtostrf(hx,10,10,hx_text);
  dtostrf(hy,10,10,hy_text);
  dtostrf(hz,10,10,hz_text);
  dtostrf(ro,10,10,ro_text);
  dtostrf(pi,10,10,pi_text);
  dtostrf(ya,10,10,ya_text);
  dtostrf(roa,10,10,roa_text);
  dtostrf(pia,10,10,pia_text);
  dtostrf(yaa,10,10,yaa_text);

  char text[500];
  snprintf(text, 500, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%u,%u,%u", 
  ax_text, ay_text, az_text, gx_text, gy_text, gz_text, hx_text, hy_text, hz_text,
   ro_text, pi_text, ya_text, roa_text, pia_text, yaa_text, 
   cal_sys_text, cal_acc_text, cal_gyr_text, cal_mag_text);
Serial.println(text);


//   Serial.print(euler.x()); //yaw
//   Serial.print(",");
//   Serial.print(euler.z()); //pitch 
//   Serial.print(",");
//   Serial.print(euler.y()); //roll

  
//   Serial.print(",");
//   Serial.print(acc.x());
//   Serial.print(",");
//   Serial.print(acc.y());
//   Serial.print(",");
//   Serial.print(acc.z());

  
//   Serial.print(",");
//   Serial.print(gyr.x());
//   Serial.print(",");
//   Serial.print(gyr.y());
//   Serial.print(",");
//   Serial.print(gyr.z());

  
//   Serial.print(",");
//   Serial.print(hag.x());
//   Serial.print(",");
//   Serial.print(hag.y());
//   Serial.print(",");
//   Serial.print(hag.z());
// //  // Quaternion data/
// //  imu::Quaternion quat = bno.getQuat();
// //  Serial.print("qW: ");
// //  Serial.print(quat.w(), 4);
// //  Serial.print(" qX: ");
// //  Serial.print(quat.x(), 4);
// //  Serial.print(" qY: ");
// //  Serial.print(quat.y(), 4);
// //  Serial.print(" qZ: ");
// //  Serial.print(quat.z(), 4);
// //  Serial.print(" ");
// /**************************************************************************/
// //! Choose 6  DoF or 9 DoF. Mahony only use 6 DoF

//   Serial.print(",");
//   Serial.print(yaw);
//   Serial.print(",");
//   Serial.print(pitch);
//   Serial.print(",");
//   Serial.print(roll);
//   Serial.print(",");
// /**************************************************************************/
//   /* Display calibration status for each sensor. */

//   Serial.print(",");
//   Serial.print(system, DEC);
//   Serial.print(",");
//   Serial.print(gyro, DEC);
//   Serial.print(",");
//   Serial.print(accel, DEC);
//   Serial.print(",");
//   Serial.println(mag, DEC);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
