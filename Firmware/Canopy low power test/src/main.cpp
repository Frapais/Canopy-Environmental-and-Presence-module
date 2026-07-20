/***************************************************************************
  ENS160 - Digital Air Quality Sensor
  
  This is an example for ENS160 basic reading in custom mode
    
  Updated by Sciosense / 25-Nov-2021
 ***************************************************************************/

#include <Wire.h>
// int ArduinoLED = 13;

//-------------------------------------------------------------
//ENS160 related items
//-------------------------------------------------------------
#include "ScioSense_ENS160.h"  // ENS160 library
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);
//ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

/*--------------------------------------------------------------------------
  SETUP function
  initiate sensor
 --------------------------------------------------------------------------*/
void setup() {

  Serial.begin(115200);

  while (!Serial) {}

  Wire.begin(2,3); //Init I2C
  //Switch on LED for init
//   pinMode(ArduinoLED, OUTPUT);
//   digitalWrite(ArduinoLED, LOW);

  Serial.println("------------------------------------------------------------");
  Serial.println("ENS160 - Digital air quality sensor");
  Serial.println();
  Serial.println("Sensor readout in custom mode");
  Serial.println();
  Serial.println("------------------------------------------------------------");
  delay(1000);

  Serial.print("ENS160...");
  bool ok = ens160.begin();
  Serial.println(ens160.available() ? "done." : "failed!");
  if (ens160.available()) {
    // Print ENS160 versions
    Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());

    Serial.print("\tSTD mode ");
    ens160.setMode(ENS160_OPMODE_STD);
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");

    // Set environment data
    ens160.set_envdata(25.0, 50.0);
  }
}

/*--------------------------------------------------------------------------
  MAIN LOOP FUNCTION
  Cylce every 1000ms and perform measurement
 --------------------------------------------------------------------------*/

void loop() {
  
  static int count = 0;
  
  if (ens160.available()) {
    bool newData = ens160.measure(false);
    if (newData) {
      Serial.print("AQI: "); Serial.print(ens160.getAQI());
      Serial.print("\tTVOC: "); Serial.print(ens160.getTVOC()); Serial.print(" ppb");
      Serial.print("\teCO2: "); Serial.print(ens160.geteCO2()); Serial.println(" ppm");
    }
  }
  
  count++;
  if (count >= 10) {
    Serial.println("Putting sensor in deep sleep for 10 seconds...");
    ens160.setMode(ENS160_OPMODE_DEP_SLEEP);
    delay(10000);
    ens160.setMode(ENS160_OPMODE_STD);
    count = 0;  // Reset counter
  }
  
  delay(1000);
}