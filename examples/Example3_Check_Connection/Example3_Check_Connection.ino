/*
  Check connection and re-initialize the sensor in order to allow hot-swapping sensors
  By: Owen Lyke
  SparkFun Electronics
  Date: May 31st 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Example3_Check_Connection

  To connect the sensor to an Arduino:
  This library supports the sensor using the I2C protocol
  On Qwiic enabled boards simply connnect the sensor with a Qwiic cable and it is set to go
  On non-qwiic boards you will need to connect 4 wires between the sensor and the host board
  (Arduino pin) = (Display pin)
  SCL = SCL on display carrier
  SDA = SDA
  GND = GND
  3.3V = 3.3V
*/

#include <SparkFun_LPS25HB_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_LPS25HB

LPS25HB pressureSensor; // Create an object of the LPS25HB class

void setup()
{
  Serial.begin(9600);
  Serial.println("LPS25HB Pressure Sensor Example 3 - Checking the Connection");
  Serial.println();

  Wire.begin();

  pressureSensor.begin(Wire, LPS25HB_I2C_ADDR_DEF); // Begin with I2C settings of your choice (see Example2_I2C_Configuration)
}

void loop()
{

  if (pressureSensor.isConnected() == true)
  {
    if (pressureSensor.getStatus() == 0x00)
    {
      pressureSensor.begin();
    } // If it is connected but not responding (for example after a hot-swap) then it may need to be re-initialized
    Serial.print("Connected. Sensor Status: ");
    Serial.print(pressureSensor.getStatus(), HEX); // Read the sensor status, the datasheet can explain what the various codes mean
    Serial.print(", Pressure (hPa): ");
    Serial.print(pressureSensor.getPressure_hPa()); // Get the pressure reading in hPa as determined by dividing the number of ADC counts by 4096 (according to the datasheet)
    Serial.print(", Temperature (degC): ");
    Serial.println(pressureSensor.getTemperature_degC()); // Get the temperature in degrees C by dividing the ADC count by 480
  }
  else
  {
    Serial.println("Disconnected");
    pressureSensor.begin();
  }
  delay(100);
}
