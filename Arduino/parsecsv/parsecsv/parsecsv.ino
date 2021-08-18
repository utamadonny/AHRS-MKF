/*  Reading csv file from SD card example for: https://github.com/michalmonday/CSV-Parser-for-Arduino  
    It's based on the following "DumpFile" example: https://www.arduino.cc/en/Tutorial/DumpFile        
    
    This example is reading "file.csv" from sd card and prints the parsed values from it.
    Example contents of the "file.csv" (used to test this example) were:
      
      column_1,column_2\n
      1,2\n
      11,22\n
      111,333\n

    This example requires SPI connection to connect with SD card as described at the "DumpFile" link.
    */

#include <CSV_Parser.h>

#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;

void setup() {
  Serial.begin(9600);
  delay(5000);

  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");


  CSV_Parser cp(/*format*/ "ffffffffff", /*has_header*/ false, /*delimiter*/ ',');
  cp.readSDfile("Book1.csv"); // this wouldn't work if SD.begin wasn't called before
  cp.parseLeftover();

  
  float   *time1 =        (float*)cp[0];
  float   *ax =           (float*)cp[1];
  float   *ay =           (float*)cp[2];
  float   *az =           (float*)cp[3];
  float   *gx =           (float*)cp[4];
  float   *gy =           (float*)cp[5];
  float   *gz =           (float*)cp[6];
  float   *mx =           (float*)cp[7];
  float   *my =           (float*)cp[8];
  float   *mz =           (float*)cp[9];
  
//  int16_t *column_1 = (int16_t*)cp["column_1"];
//  int16_t *column_2 = (int16_t*)cp["column_2"];

//  if (ax && ay && az && gx && gy && gz && mx && my && mz && time1) {
    for(int row = 0; row < cp.getRowsCount(); row++) {
        Serial.print(ax[row]);
        Serial.print(',');
        Serial.print(ay[row]);
        Serial.print(',');
        Serial.print(az[row]);
//      Serial.print("row = ");
//      Serial.print(row, DEC);
//      Serial.print(", column_1 = ");
//      Serial.print(column_1[row], DEC);
//      Serial.print(", column_2 = ");
//      Serial.println(column_2[row], DEC);
    }
//  } else {
//    Serial.println("At least 1 of the columns was not found, something went wrong.");
//  }

  // output parsed values (allows to check that the file was parsed correctly)
  cp.print(); // assumes that "Serial.begin()" was called before (otherwise it won't work)
}

void loop() {

}
