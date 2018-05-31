#include <LPS25HB.h>

LPS25HB pressure_sensor; // Create an object of the LPS25HB class

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){} 
  Serial.println("Hello!");

  pressure_sensor.begin(); // Simplest begin function uses the default Wire port and the default I2C address for the sensor
}

void loop() {
  // put your main code here, to run repeatedly:
  if(pressure_sensor.isConnected() == LPS25HB_CODE_CONNECTED)                                         // Check that the sensor is connected
  {
    if(pressure_sensor.getStatus() == 0x00){pressure_sensor.begin();}                                 // If it is connected but not responding (for example after a hot-swap) then it may need to be re-initialized
    Serial.print("Connected. Sensor Status: "); Serial.print(pressure_sensor.getStatus(),HEX);        // Read the sensor status, the datasheet can explain what the various codes mean
    Serial.print("Pressure in hPa: "); Serial.print(pressure_sensor.getPressure_hPa());               // Get the pressure reading in hPa as determined by dividing the number of ADC counts by 4096 (according to the datasheet)
    Serial.print(", Temperature (degC): "); Serial.println(pressure_sensor.getTemperature_degC());    // Get the temperature in degrees C by dividing the ADC count by 480
  }
  else
  {
    Serial.println("Disconnected");
    pressure_sensor.begin();
  }
  
  delay(100);
}
