#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

/* CSV File Reading */
File file;
int SC = 53;  //SC - Pin 53 Arduino Mega
char location;

bool readLine(File &f, char* line, size_t maxLen) {
  for (size_t n = 0; n < maxLen; n++) {
    int c = f.read();
    if ( c < 0 && n == 0) return false;  // EOF
    if (c < 0 || c == '\n') {
      line[n] = 0;
      return true;
    }
    line[n] = c;
  }
  return false; // line too float
}

bool readVals(float* ax, float* ay, float* az, float* gx, float* gy,float* gz,float* mx, float* my, float* mz) {
  char line[200], *ptr, *str;
  if (!readLine(file, line, sizeof(line))) {
    return false;  // EOF or too float
  }
  *ax = strtol(line, &ptr, 10);
  if (ptr == line) return false;  // bad number if equal
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *ay = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *az = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *gx = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *gy = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *gz = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *mx = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *my = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *mz = strtol(ptr, &str, 10);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
//   String a = strtok_r(ptr, ",", &str);
//   String first(str);
//   *loc = first;
//   String let(a);
//   *loc2 = let; 
//   return str != ptr;  // true if number found
}
/* Close CSV File Reading */

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  //SD Card Reader Setup
  Serial.begin(9600);
  if (!SD.begin(SC)) {
    Serial.println("begin error");
    return;
  }
  file = SD.open("gps.CSV", FILE_READ);
  if (!file) {
    Serial.println("open error");
    return;
  }
  
}

void loop() // run over and over
{ 
floast ax, ay, az, gx,gy,gz,mx,my,mz;
String loc,loc2;
  while (readVais(&ax, &ay, &az, &gx, &gy, &gz,&mx,&my,&mz)) {
    
    //First 4 float datatype variables 
    Serial.println(ax);
    Serial.println(ay);
    Serial.println(az);
//     Serial.println(k);
//     //Last 2 String type variables
//     Serial.println(loc);
//     Serial.println(loc2);
  } 
 
}