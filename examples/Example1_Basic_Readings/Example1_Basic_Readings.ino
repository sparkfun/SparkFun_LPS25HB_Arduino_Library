/*
  Take pressure and temperature readings with the LPS25HB using I2C
  By: Owen Lyke
  SparkFun Electronics
  Date: May 31st 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Example1_Basic_Readings

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


#include <SparkFun_LPS25HB_Arduino_Library.h>  // Click here to get the library: http://librarymanager/All#SparkFun_LPS25HB

LPS25HB pressureSensor; // Create an object of the LPS25HB class

void setup() {
  Serial.begin(9600);
  Serial.println("LPS25HB Pressure Sensor Example 1 - Basic Readings");
  Serial.println();

  pressureSensor.begin();    // Begin links an I2C port and I2C address to the sensor, sets an I2C speed, begins I2C on the main board, and then sets default settings

  if(pressureSensor.isConnected() == false)  // The library supports some different error codes such as "DISCONNECTED"
  {
    Serial.println("LPS25HB disconnected. Reset the board to try again.");        // Alert the user that the device cannot be reached
    Serial.println("Are you using the right Wire port and I2C address?");         // Suggest possible fixes
    Serial.println("See Example2_I2C_Configuration for how to change these.");    // Suggest possible fixes
    Serial.println("");
    while(1);
  }
}

void loop() {
    Serial.print("Pressure in hPa: "); 
    Serial.print(pressureSensor.getPressure_hPa());          // Get the pressure reading in hPa
    Serial.print(", Temperature (degC): "); 
    Serial.println(pressureSensor.getTemperature_degC());    // Get the temperature in degrees C

    delay(40);                                               // Wait - 40 ms corresponds to the maximum update rate of the sensor (25 Hz)
}
