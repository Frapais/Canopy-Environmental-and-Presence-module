/***************************************************************************
  ENS160 - Digital Air Quality Sensor
  
  This is an example for ENS160 basic reading in standard mode
    
  Updated by Sciosense / 25-Nov-2021
 ***************************************************************************/

#include <Wire.h>

#include "ScioSense_ENS160.h"  // ENS160 library
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);
//ScioSense_ENS160      ens160(ENS160_I2CADDR_1);
unsigned long now = 0;
/*--------------------------------------------------------------------------
  SETUP function
  initiate sensor
 --------------------------------------------------------------------------*/
void setup() {

  Serial.begin(9600);

  while (!Serial) {}

  Serial.println("------------------------------------------------------------");
  Serial.println("ENS160 - Digital air quality sensor");
  Serial.println();
  Serial.println("Sensor readout in standard mode");
  Serial.println();
  Serial.println("------------------------------------------------------------");
  delay(1000);
  // Wire.begin(2,3);
  Serial.print("ENS160...");
  ens160.setI2C(2, 3);
  ens160.begin(true);
  
  Serial.println(ens160.available() ? "done." : "failed!");
  if (ens160.available()) {
    // Print ENS160 versions
    Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());
  
    Serial.print("\tStandard mode ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "std." : "failed!");
    // ens160.set_envdata(25.0, 50.0);
    ens160.set_envdata210(uint16_t(31.0), uint16_t(30.0));
    now = millis();
  }
}

/*--------------------------------------------------------------------------
  MAIN LOOP FUNCTION
  Cylce every 1000ms and perform measurement
 --------------------------------------------------------------------------*/

void loop() {

  if(millis() - now > 1000)
  {
    now = millis();
    int misr = ens160.getMISR();
    Serial.printf("STATUS: %d\n", misr);
    Serial.println("started");
    if(ens160.available())
    {
      ens160.measure(false);
      Serial.print("AQI: ");Serial.print(ens160.getAQI());Serial.print("\t");
      Serial.print("TVOC: ");Serial.print(ens160.getTVOC());Serial.print("ppb\t");
      Serial.print("eCO2: ");Serial.print(ens160.geteCO2());Serial.print("ppm\t");
    }
    else
    {
      Serial.println("ENS160 not available");
    }
  }
  delay(1);
}
