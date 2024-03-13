/******************************************************************************
  air_velocity_testing
  based on Sparkfun example: Example_01_BasicReadings.ino

******************************************************************************/

#include <Wire.h>
#include <SparkFun_FS3000_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_FS3000

FS3000 fs;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Set the range to match which version of the sensor you are using.
  //   FS3000-1005 (0-7.23 m/sec) --->>>  AIRFLOW_RANGE_7_MPS
  //   FS3000-1015 (0-15 m/sec)   --->>>  AIRFLOW_RANGE_15_MPS

  //fs.setRange(AIRFLOW_RANGE_7_MPS);
  fs.setRange(AIRFLOW_RANGE_15_MPS); 
}

void loop()
{
    Serial.print(fs.readMetersPerSecond()); // note, this returns a float from 0-7.23 for the FS3000-1005, and 0-15 for the FS3000-1015 
    Serial.print('\n');
    delay(1000); // note, reponse time on the sensor is 125ms
}
