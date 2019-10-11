/*
  Change the Wire port and I2C address to enable using additional sensors
  By: Owen Lyke
  SparkFun Electronics
  Date: May 31st 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Example2_I2C_Configuration

  Hardware requirements:
  If you want to use a second Wire port first ensure that the board you are using supports it.
  If you want to use two LPS25HB sensors on the same Wire port then ensure that the ADR jumper
  is soldered on one and only one of the sensors.

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
  while (!Serial)
  {
  }
  Serial.println("LPS25HB Pressure Sensor Example 2 - Configuring I2C Past Default");
  Serial.println();

  Wire.begin();
  Wire.setClock(400000);

  /* Using the begin() function for the LPS25HB gives the opportunity to 
   *  change which Wire port and I2C address you want to use. This makes
   *  it possible to use up to 2 sensors on any one I2C bus, and to put 
   *  additional sensors on other Wire ports if the microcontroller supports
   *  that.
   */

  pressureSensor.begin(Wire, LPS25HB_I2C_ADDR_DEF); // Begin using default values when using the Qwiic system with the LPS25HB right out of the box
                                                    //  pressureSensor.begin(Wire2);                                // By passing in only one argument you specify which Wire port to use - here we specify the Wire2 port
                                                    //  pressureSensor.begin(Wire, LPS25HB_I2C_ADDR_ALT);           // In order to use the alternate address you *must* pass in both parameters like this
                                                    //  pressureSensor.begin(LPS25HB_I2C_ADDR_ALT);                 // This line would fail because begin() expects the first argument to be a Wire port, not an I2C address

  if (pressureSensor.isConnected() == false) // The library supports some different error codes such as "DISCONNECTED"
  {
    Serial.println("LPS25HB not found with your settings: "); // Alert the user that the device cannot be reached
    Serial.println("Troubleshooting: ");
    Serial.println("  Ensure the correct I2C address is used (in begin() function)");
    Serial.println("  Ensure the correct Wire port is used (in begin() function)");
    Serial.println("  Ensure the sensor is connected correctly");
    Serial.println("Reset the board or re-upload to try again.");
    Serial.println("");
    while (1)
      ;
  }
}

void loop()
{
  Serial.print("Pressure (hPa): ");
  Serial.print(pressureSensor.getPressure_hPa()); // Get the pressure reading in hPa
  Serial.print(", Temperature (degC): ");
  Serial.println(pressureSensor.getTemperature_degC()); // Get the temperature in degrees C

  delay(40); // Wait - 40 ms corresponds to the maximum update rate of the sensor (25 Hz)
}
