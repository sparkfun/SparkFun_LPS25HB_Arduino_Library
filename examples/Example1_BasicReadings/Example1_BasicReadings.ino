
// Click here to get the library: http://librarymanager/All#SparkFun_VEML6075 /// Update<---
#include <LPS25HB.h>

LPS25HB pressure_sensor; // Create an object of the LPS25HB class

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello!");

  pressure_sensor.begin(); // Simplest begin function uses the default Wire port and the default I2C address for the sensor
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.print("Pressure in hPa: "); Serial.print(pressure_sensor.getPressure_hPa());               // Get the pressure reading in hPa as determined by dividing the number of ADC counts by 4096 (according to the datasheet)
    Serial.print(", Temperature (degC): "); Serial.println(pressure_sensor.getTemperature_degC());    // Get the temperature in degrees C by dividing the ADC count by 480

  
  delay(100);
}
