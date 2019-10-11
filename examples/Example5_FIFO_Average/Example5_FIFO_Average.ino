/*
  Use the FIFO buffer in the LSP25HB barometric pressure sensor to implement a hardware moving average of samples
  By: Owen Lyke
  SparkFun Electronics
  Date: June 4th 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Example3_FIFOAverage

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
  Serial.println("LPS25HB Pressure Sensor Example 5 - FIFO Averaging");
  Serial.println();

  Wire.begin();
  pressureSensor.begin(); // Begin links an I2C port and I2C address to the sensor, and begins I2C on the main board

  while (pressureSensor.isConnected() == false)
  {
    Serial.print("Waiting for connection to LPS25HB.");                         // Alert the user that the device cannot be reached
    Serial.print(" Are you using the right Wire port and I2C address?");        // Suggest possible fixes
    Serial.println(" See Example2_I2C_Configuration for how to change these."); // Example2 illustrates how to change the Wire port or I2C address
    delay(250);
  }

  pressureSensor.setFIFOMeanNum(LPS25HB_FIFO_CTRL_M_32); // Specifies the desired number of moving average samples. Valid values are 2, 4, 8, 16, and 32
  pressureSensor.setFIFOMode(LPS25HB_FIFO_CTRL_MEAN);    // Sets the FIFO to the MEAN mode, which implements a hardware moving average
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
    Serial.print("Pressure in hPa: ");
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
