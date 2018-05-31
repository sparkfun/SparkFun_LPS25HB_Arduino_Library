/*
  Use the FIFO buffer in the LSP25HB barometric pressure sensor to implement a hardware moving average of samples
  By: Owen Lyke
  SparkFun Electronics
  Date: May 31st 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Example3_FIFOAverage

  To connect the sensor to an Arduino:
  This library supports the sensor using the I2C protocol
  (Arduino pin) = (Display pin)
  Pin 13 = SCLK on display carrier
  11 = SDIN
  10 = !CS
  9 = !RES

  The display is 160 pixels long and 32 pixels wide
  Each 4-bit nibble is the 4-bit grayscale for that pixel
  Therefore each byte of data written to the display paints two sequential pixels
  Loops that write to the display should be 80 iterations wide and 32 iterations tall
*/




#include <LPS25HB.h>

LPS25HB pressureSensor; // Create an object of the LPS25HB class

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){} 
  Serial.println("Hello!");

  pressureSensor.begin();                                      // Simplest begin function uses the default Wire port and the default I2C address for the sensor
  pressureSensor.setFIFOMeanNum(LPS25HB_FIFO_CTRL_M_32);       // Specifies the desired number of moving average samples. Valid values are 2, 4, 8, 16, and 32
  pressureSensor.setFIFOMode(LPS25HB_FIFO_CTRL_MEAN);          // Sets the FIFO to the MEAN mode, which implements a hardware moving average
  
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.print("ADC Counts: "); Serial.println(pressure_sensor.getPressure_raw(),HEX);

  if(pressureSensor.isConnected() == LPS25HB_CODE_CONNECTED)
  {
    if(pressureSensor.getStatus() == 0x00){pressureSensor.begin();}                                 // If it is connected but not responding (for example after a hot-swap) then it may need to be re-initialized
    Serial.print("Connected. Sensor Status: "); 
    Serial.print(pressureSensor.getStatus(),HEX);        // Read the sensor status, the datasheet can explain what the various codes mean
    Serial.print("Pressure in hPa: "); 
    Serial.print(pressureSensor.getPressure_hPa());               // Get the pressure reading in hPa as determined by dividing the number of ADC counts by 4096 (according to the datasheet)
    Serial.print(", Temperature (degC): "); 
    Serial.println(pressureSensor.getTemperature_degC());    // Get the temperature in degrees C by dividing the ADC count by 480
  }
  else
  {
    Serial.println("Disconnected");
    pressureSensor.begin();
  }
  delay(100);
}
