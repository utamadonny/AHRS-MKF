#include <Wire.h>   //inisialisasi dipendencies
#include <Adafruit_SSD1306.h> // library OLED
#include <SPI.h>    // SPI 
#include "SdFat.h"  // SD card
#include "MPU9250.h"
#include "Plotter.h"
//=================================================//
double ax,ay,az,gx,gy,gz,mx,my,mz;
Plotter p;
#define OLED_RESET 4 //pin OLED 4
Adafruit_SSD1306 display(OLED_RESET);
MPU9250 IMU(Wire, 0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
int status;
unsigned long previousMillis = 0;   // set timer dengan fungsi milis (milisecond) ms
unsigned long interval = 100;   // set timer untuk bekerja tiap intervar 100ms
const int chipSelect = 10;     // pin  sd card  CS
SdFat SD;
File TimeFile;    //nama file yang mau disimpan
File AccelFile;   //nama file yang mau disimpan
File GyroFile;   //nama file yang mau disimpan
File MagnetFile;

void setup() {
  plotsetup();
  SD.begin(chipSelect);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {
  }
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  //DataLogger
  unsigned long currentMillis = millis();       // timer dengan fungsi milis current time merupakan fungsi milis
  if (currentMillis - previousMillis >= interval) // ketika waktu currentMillis - previousMillis >= interval maka eksekusi penyimpanan file
  {
    previousMillis = currentMillis;    // ubah previousMillis=currentMillis supaya mereset nilai

    TimeFile = SD.open("TIME.txt", FILE_WRITE);   // simpan file
    if (TimeFile) {
      TimeFile.println(currentMillis);
      TimeFile.close();
    }
    AccelFile = SD.open("Accel.txt", FILE_WRITE);    //simpan file
    if (AccelFile) {
      AccelFile.print(IMU.getAccelX_mss(), 4);
      AccelFile.print(" ");
      AccelFile.print(IMU.getAccelY_mss(), 4);
      AccelFile.print(" ");
      AccelFile.print(IMU.getAccelZ_mss(), 4);
      AccelFile.println();
      AccelFile.close();
    }
    GyroFile = SD.open("Gyro.txt", FILE_WRITE);      //simpan file
    if (GyroFile) {
      GyroFile.print(IMU.getGyroX_rads(), 4);
      GyroFile.print(" ");
      GyroFile.print(IMU.getGyroY_rads(), 4);
      GyroFile.print(" ");
      GyroFile.print(IMU.getGyroZ_rads(), 4);
      GyroFile.println();
      GyroFile.close();
    }
    MagnetFile = SD.open("Magneto.txt", FILE_WRITE);      //simpan file
    if (MagnetFile) {
      MagnetFile.print(IMU.getMagX_uT(), 4);
      MagnetFile.print(" ");
      MagnetFile.print(IMU.getMagY_uT(), 4);
      MagnetFile.print(" ");
      MagnetFile.print(IMU.getMagZ_uT(), 4);
      MagnetFile.println();
      MagnetFile.close();
    }
    displaydata();    //tampilkan pada layar OLED
    IMUserialdata(); //Show Serial DataLogger
    plotloop();
  }
}

void IMUserialdata() {
  // display the data
  Serial.print(IMU.getAccelX_mss(), 4);
  Serial.print(" ");
  Serial.print(IMU.getAccelY_mss(), 4);
  Serial.print(" ");
  Serial.print(IMU.getAccelZ_mss(), 4);
  Serial.print(" ");
  Serial.print(IMU.getGyroX_rads(), 4);
  Serial.print(" ");
  Serial.print(IMU.getGyroY_rads(), 4);
  Serial.print(" ");
  Serial.print(IMU.getGyroZ_rads(), 4);
  Serial.print(" ");
  Serial.print(IMU.getMagX_uT(), 4);
  Serial.print(" ");
  Serial.print(IMU.getMagY_uT(), 4);
  Serial.print(" ");
  Serial.print(IMU.getMagZ_uT(), 4);
  Serial.println();
}

void displaydata() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  //display.println(loadvoltage);
  display.setCursor(35, 0);
  display.println("V");
  display.setCursor(50, 0);
  //display.println(current_mA);
  display.setCursor(95, 0);
  display.println("mA");
  display.setCursor(0, 10);
  //display.println(loadvoltage * current_mA);
  display.setCursor(65, 10);
  display.println("mW");
  display.setCursor(0, 20);
  //display.println(energy);
  display.setCursor(65, 20);
  display.println("mWh");
  display.display();
}

void plotsetup(){
  p.Begin();
  p.AddTimeGraph("Accelero", 1000, "ax", ax, "ay", ay, "az", az);
  p.AddTimeGraph ("Gyro", 1000, "gx",gx,"gy",gy,"gz",gz);
  p.AddTimeGraph ("Magneto", 1000, "mx", mx, "my",my ,"mz",mz);
}

void plotloop(){
  ax=(IMU.getAccelX_mss(), 4);
  ay=(IMU.getAccelY_mss(), 4);
  az=(IMU.getAccelZ_mss(), 4);
  gx=(IMU.getGyroX_rads(), 4);
  gy=(IMU.getGyroY_rads(), 4);
  gz=(IMU.getGyroZ_rads(), 4);
  mx=(IMU.getMagX_uT(), 4);
  my=(IMU.getMagY_uT(), 4);
  mz=(IMU.getMagZ_uT(), 4);

  p.Plot();
}

