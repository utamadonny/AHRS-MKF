#include "DFRobot_BNO055.h"
#include "Wire.h"

typedef DFRobot_BNO055_IIC BNO; // ******** use abbreviations instead of full names ********

BNO bno(&Wire, 0x28); // input TwoWire interface and IIC address

// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch (eStatus)
  {
  case BNO::eStatusOK:
    Serial.println("everything ok");
    break;
  case BNO::eStatusErr:
    Serial.println("unknow error");
    break;
  case BNO::eStatusErrDeviceNotDetect:
    Serial.println("device not detected");
    break;
  case BNO::eStatusErrDeviceReadyTimeOut:
    Serial.println("device ready time out");
    break;
  case BNO::eStatusErrDeviceStatus:
    Serial.println("device internal status error");
    break;
  default:
    Serial.println("unknow status");
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  bno.reset();
  while (bno.begin() != BNO::eStatusOK)
  {
    Serial.println("bno begin faild");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");
}

// #define printAxisData(sAxis) \
//   Serial.print(","); \
//   Serial.print(sAxis.x); \
//   Serial.print(","); \
//   Serial.print(sAxis.y); \
//   Serial.print(","); \
//   Serial.print(sAxis.z)

void loop()
{
  BNO::sAxisAnalog_t sAccAnalog, sMagAnalog, sGyrAnalog, sLiaAnalog, sGrvAnalog;
  BNO::sEulAnalog_t sEulAnalog;
  BNO::sQuaAnalog_t sQuaAnalog;
  sAccAnalog = bno.getAxis(BNO::eAxisAcc); // read acceleration
  sMagAnalog = bno.getAxis(BNO::eAxisMag); // read geomagnetic
  sGyrAnalog = bno.getAxis(BNO::eAxisGyr); // read gyroscope
  sLiaAnalog = bno.getAxis(BNO::eAxisLia);    // read linear acceleration
  sGrvAnalog = bno.getAxis(BNO::eAxisGrv);    // read gravity vector
  sEulAnalog = bno.getEul();                  // read euler angle
  sQuaAnalog = bno.getQua();                  // read quaternionzzzzz
                                           //  Serial.println();
                                           //  Serial.println("======== analog data print start ========");
                                           //  Serial.print("acc analog: (unit mg)       "); printAxisData(sAccAnalog);
                                           //  Serial.print("mag analog: (unit ut)       "); printAxisData(sMagAnalog);
                                           //  Serial.print("gyr analog: (unit dps)      "); printAxisData(sGyrAnalog);
                                           //  Serial.print("lia analog: (unit mg)       "); printAxisData(sLiaAnalog);
                                           //  Serial.print("grv analog: (unit mg)       "); printAxisData(sGrvAnalog);
                                           //  Serial.print("eul analog: (unit degree)   "); Serial.print(" head: "); Serial.print(sEulAnalog.head); Serial.print(" roll: "); Serial.print(sEulAnalog.roll);  Serial.print(" pitch: "); Serial.println(sEulAnalog.pitch);
                                           //  Serial.print("qua analog: (no unit)       "); Serial.print(" w: "); Serial.print(sQuaAnalog.w); printAxisData(sQuaAnalog);
                                           //  Serial.println("========  analog data print end  ========");

  //  printAxisData(sAccAnalog); //1 m/s
  //  printAxisData(sGyrAnalog); // 1 rps
  //  printAxisData(sMagAnalog); // 1 uTs

  // Serial.print(",");
  Serial.print(sAccAnalog.x);
  Serial.print(F(", "));
  Serial.print(sAccAnalog.y);
  Serial.print(F(", "));
  Serial.print(sAccAnalog.z);
  Serial.print(F(", "));
  Serial.print(sGyrAnalog.x);
  Serial.print(F(", "));
  Serial.print(sGyrAnalog.y);
  Serial.print(F(", "));
  Serial.print(sGyrAnalog.z);
  Serial.print(F(", "));
  Serial.print(sMagAnalog.x);
  Serial.print(F(", "));
  Serial.print(sMagAnalog.y);
  Serial.print(F(", "));
  Serial.print(sMagAnalog.z);
  Serial.println(F(""));

  delay(100);
}
