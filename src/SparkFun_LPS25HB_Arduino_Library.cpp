/*

SparkFun_LPS25HB_Arduino_Library.cpp

Header file: SparkFun_LPS25HB_Arduino_Library.h

Created: May 2018
Last Updated: June 2018

Authors:
Owen Lyke

*/

#include <SparkFun_LPS25HB_Arduino_Library.h>  // Click here to get the library: http://librarymanager/All#SparkFun_LPS25HB


/**
   * Constructor for an object of the LPS25HB class
*/
LPS25HB::LPS25HB( void )
{

}

/**
   * Allows a user to specify which Wire (I2C) port, sensor I2C address, and I2C speed before beginning I2C
   * The desired Wire port and I2C address are then associated with the object until modified by the user
   * The clock frequency is applied but can be changed again externally
   * @return Boolean, true if connected and false if disconnected
*/
bool LPS25HB::begin( TwoWire &wirePort, uint8_t address, uint32_t clock_frequency)
{
	sensor_address = address;

	_i2cPort = &wirePort;
	_i2cPort->begin();
	_i2cPort->setClock(clock_frequency);

	if(isConnected() != true)
	{
		return false;
	}

	// Now setup default values
	uint8_t values[5];

	values[0] = LPS25HB_RES_CONF_DEFAULT;									// Set the resolution configuration to default
	write( LPS25HB_REG_RES_CONF, values, 1 );

	values[0] = LPS25HB_CTRL_REG1_PD_ACTIVE | LPS25HB_CTRL_REG1_ODR_25HZ ;	// Turn the sensor ON and set output rate to 25 Hz
	values[1] = LPS25HB_CTRL_REG2_DEFAULT;									// Default
	values[2] = LPS25HB_CTRL_REG3_INT_L | LPS25HB_CTRL_REG3_OD ;			// Set interrupts to output LOW and Open Drain function	
	values[3] = LPS25HB_CTRL_REG4_DEFAULT;									// Default
	values[4] = LPS25HB_INTERRUPT_CFG_DEFFAULT;								// Set the Interrupt CFG register to default
	write( LPS25HB_REG_CTRL_REG1, values, 5 );				// Write the values consecutively to the device

	return true;
}




/**
   * Determines if a LPS25HB device is available at the address associated with the object 
   * @return Boolean, true if connected and false if disconnected
*/
bool LPS25HB::isConnected()
{
	_i2cPort->beginTransmission(sensor_address);						// Check the desired address
  	if(_i2cPort->endTransmission() == 0)
  	{
  		if(getID() == LPS25HB_DEVID)
  		{
  			lastCode = LPS25HB_CODE_CONNECTED;								// If connected then set code to connected
  			return true;
  		}
 
  		lastCode = LPS25_HB_CODE_WRONG_ID;
  		return false;
  	}

  	lastCode = LPS25HB_CODE_NO_DEV_AT_ADDRESS;													// Store the code
  	return false;														// Return value
}

/**
   * Gets the device ID from the WHO_AM_I register. The return should be equal to 0xBD for valid LPS25HB devices
   * @return Device ID
*/
uint8_t LPS25HB::getID()
{
	uint8_t retval = 0x00;											
	read( LPS25HB_REG_WHO_AM_I, &retval, 1 );	// Perform the read and store the code returned
	return retval;														// Return the read value
}

/**
   * Gets the status code from the STATUS_REG register of the device
   * @return Status code
*/
uint8_t LPS25HB::getStatus()
{
	uint8_t retval = 0x00;
	read( LPS25HB_REG_STATUS_REG, &retval, 1 );
	return retval;
}













/**
   * Gets the 16 bit temperature reading in ADC counts
   * @return Temperature reading
*/
int16_t		LPS25HB::getTemperature_raw()
{
	uint8_t data[2];
	read(LPS25HB_REG_TEMP_OUT_L, data, 2 );
	int16_t retval = 0x00;
	// retval = ((data[1] << 8) | (data[0] << 0));
	retval = (data[1] << 8 | data[0]);
	return retval;
}

/**
   * Gets the temperature reading in deg C
   * @return Temperature reading
*/
float		LPS25HB::getTemperature_degC()
{
	int16_t raw = getTemperature_raw();
	return (float)(raw/480.0);
}

/**
   * Gets the 24 bit pressure reading in ADC counts
   * @return Pressure reading
*/
int32_t 	LPS25HB::getPressure_raw()
{
	uint8_t data[3];
	read(LPS25HB_REG_PRESS_OUT_XL, data, 3 );
	int32_t retval = 0x00;
	retval = ((int32_t)data[0] << 0) | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16);
	if(data[2] & 0x80){retval |= 0xFF000000; } //This formats the 32 bit data type to twos complement
	return retval;
}

/**
   * Gets the pressure reading in hPa
   * @return Pressure reading
*/
float		LPS25HB::getPressure_hPa()
{
	int32_t raw = getPressure_raw();
	return (float)(raw/4096.0);
}




















/**
   * Sets a threshold number of ADC counts that is used to trigger interrupts and flags on temperature
   * @param avg_code A byte that specifies the number of averages. Suggest using defined values in .h file
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::setReferencePressure(uint32_t adc_val)
{
	uint8_t data[3];
	data[0] = ((adc_val & 0x0000FF) >> 0);
	data[1] = ((adc_val & 0x00FF00) >> 8);
	data[2] = ((adc_val & 0xFF0000) >> 16);
	write( LPS25HB_REG_REF_P_XL, data, 3 );
}

/**
   * Sets a threshold number of ADC counts that is used to trigger interrupts and flags on pressure
   * @param avg_code A byte that specifies the number of averages. Suggest using defined values in .h file
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::setPressureThreshold(uint16_t adc_val)
{
	uint8_t data[2];
	data[0] = ((adc_val & 0x00FF) >> 0);
	data[1] = ((adc_val & 0xFF00) >> 8);
	write( LPS25HB_REG_THS_P_L, data, 2 );
}

/**
   * This sets the resolution of temperature sensor by changing the number of
   * readings that are averaged for each result update
   * @param avg_code A byte that specifies the number of averages. Suggest using defined values in .h file
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::setTemperatureAverages(uint8_t avg_code)
{
	return applySetting(LPS25HB_REG_RES_CONF, (avg_code & 0x0C));
}

/**
   * This sets the resolution of pressure sensor by changing the number of
   * readings that are averaged for each result update
   * @param avg_code A byte that specifies the number of averages. Suggest using defined values in .h file
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::setPressureAverages(uint8_t avg_code)
{
	return applySetting(LPS25HB_REG_RES_CONF, (avg_code & 0x03));
}

/**
   * Sets the sensor's output data rate according to datasheet option. 
   * Note that this is the refresh rate of the values in internal registers 
   * because as a slave I2C device it may not push data out on its own.
   * @param odr_code A byte that specifies the output data rate. Suggest using defined values in .h file
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::setOutputDataRate(uint8_t odr_code)
{
	return applySetting(LPS25HB_REG_CTRL_REG1, (odr_code & 0x70));
}

/**
   * Sets the FIFO mode according to the datasheet options
   * @param mode_code A byte that specifies the FIFO mode. Suggest using defined values in .h file
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::setFIFOMode(uint8_t mode_code)
{
	uint8_t prev_setting;
	uint8_t new_setting;
	read(LPS25HB_REG_CTRL_REG2, &prev_setting, 1 );

	if(mode_code == LPS25HB_FIFO_CTRL_BYPASS)
	{
		new_setting = (prev_setting & (~LPS25HB_CTRL_REG2_FIFO_EN));			// Clears the FIFO enable bit
		if(write(LPS25HB_REG_CTRL_REG2, &new_setting, 1 ) != LPS25HB_CODE_NOM){return LPS25HB_CODE_ERR;}
	}
	else
	{
		new_setting = (prev_setting | LPS25HB_CTRL_REG2_FIFO_EN);			// Enables the FIFO
		if(write(LPS25HB_REG_CTRL_REG2, &new_setting, 1 ) != LPS25HB_CODE_NOM){return LPS25HB_CODE_ERR;}
	}

	read(LPS25HB_REG_FIFO_CTRL, &prev_setting, 1 );
	new_setting = ((prev_setting & 0x1F) | (mode_code & 0xE0));
	if(write(LPS25HB_REG_FIFO_CTRL, &new_setting, 1 ) != LPS25HB_CODE_NOM){return LPS25HB_CODE_ERR;}			// Sets the new FIFO mode
	return LPS25HB_CODE_NOM;

}

/**
   * Sets the number of hardware averages performed by the FIFO buffer to one of several allowed values
   * @param num_code A byte that specifies how many hardware averags to use according to the datasheet. Suggest using defined register options in .h file
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::setFIFOMeanNum(uint8_t num_code)
{
	return applySetting(LPS25HB_REG_FIFO_CTRL, (num_code & 0x1F));
}









/**
   * A member that allows setting individual bits within a given register of the device
   * @param reg_adr The address of the register to modify. Use of the LPS25HB_RegistersTypeDef enumeration values is suggested.
   * @param setting A byte wide bitmask indicating which bits to set. Use of the defined setting values in the .h file is suggested
   * @see LPS25HB_RegistersTypeDef
   * @see Defined setting values in .h file
   * @return Boolean, true if successful and false if unsuccessful
*/
bool LPS25HB::applySetting(uint8_t reg_adr, uint8_t setting)
{
	uint8_t data;																// Declare space for the data
	read(reg_adr, &data, 1 );									// Now fill that space with the old setting from the sensor
	data |= setting;															// OR in the new setting, preserving the other fields
	if(write(reg_adr, &data, 1 ) != LPS25HB_CODE_NOM)			// Write the new data back to the device and make sure it was successful
	{
		lastCode = LPS25HB_CODE_SET_FAIL;
		return false;												// If it failed then return an error
	}
	lastCode = LPS25HB_CODE_NOM;
	return true;													// Otherwise its all good!
}

/**
   * A member that allows resetting individual bits within a given register of the device
   * @param reg_adr The address of the register to modify. Use of the LPS25HB_RegistersTypeDef enumeration values is suggested.
   * @param setting A byte wide bitmask indicating which bits to reset. Use of the defined setting values in the .h file is suggested
   * @see LPS25HB_RegistersTypeDef
   * @see Defined setting values in .h file
   * @return Boolean, true if read was successful and false if unsuccessful
*/
bool LPS25HB::removeSetting(uint8_t reg_adr, uint8_t setting)
{
	uint8_t data;																// Declare space for the data
	read(reg_adr, &data, 1 );									// Now fill that space with the old setting from the sensor
	data &= ~setting;															// AND in the opposite of the setting value to reset the desired bit(s) while leaving others intact
	if(write(reg_adr, &data, 1 ) != LPS25HB_CODE_NOM)			// Write the new data back to the device and make sure it was successful
	{
		lastCode = LPS25HB_CODE_RESET_FAIL;
		return false;												// If it failed then return an error
	}
	lastCode = LPS25HB_CODE_NOM;
	return true;													// Otherwise its all good!
}









/**
   * A member that allows reading a sequence of data from consecutive register locations in the device
   * @param reg_adr The address of the first register to read from. Use of the LPS25HB_RegistersTypeDef enumeration values is suggested.
   * @param pdata A pointer to the first element of the buffer that will hold the returned data
   * @param size The number of elements to read. I.e. to read only one byte to the device use size=1
   * @see LPS25HB_RegistersTypeDef
   * @return Boolean, true if read was successful and false if unsuccessful
*/
bool LPS25HB::read( uint8_t reg_adr, uint8_t * pdata, uint8_t size)
{
	bool retval = false;

	_i2cPort->beginTransmission( sensor_address );				// Begin talking to the desired sensor
	_i2cPort->write(reg_adr | (1<<7));

	if (_i2cPort->endTransmission(false) != 0) 			//Send a restart command. Do not release bus.
  	{
    	lastCode = LPS25HB_CODE_ERR; 						//Sensor did not ACK
    	return false;
  	}

  	_i2cPort->requestFrom(sensor_address, size);				// Default value of true used here to release bus once complete  	
  	for(uint8_t indi = 0; indi < size; indi++)
  	{
  		if(_i2cPort->available())
  		{
  			*(pdata + indi) = _i2cPort->read();
  		}
  		else
  		{
  			lastCode = LPS25HB_CODE_RX_UNDERFLOW; 						//Sensor did not ACK
    		return false;
  		}
  	}		
	
	lastCode = LPS25HB_CODE_NOM;
	return true;								
}

/**
   * A member that allows writing a sequence of data to consecutive register locations in the device
   * @param reg_adr The address of the first register to write to. Use of the LPS25HB_RegistersTypeDef enumeration values is suggested.
   * @param pdata A pointer to the first element of the buffer that contains the data to be written
   * @param size The number of elements to write. I.e. to write only one byte to the device use size=1
   * @see LPS25HB_RegistersTypeDef
   * @return Boolean, true if write was successful and false if unsuccessful
*/	
bool LPS25HB::write( uint8_t reg_adr, uint8_t * pdata, uint8_t size)
{
	bool retval = false;

	_i2cPort->beginTransmission( sensor_address );		// Begin talking to the desired sensor
	_i2cPort->write(reg_adr | (1<<7));					// Specify a write to the desried register with the mutli-byte bit set
	_i2cPort->write(pdata, size);						// Write all the bytes
	
	if (_i2cPort->endTransmission() == 0)
  	{
    	retval = true; 						// Sensor did ACK
  	}

  	lastCode = LPS25HB_CODE_NOM;
  	return retval;
}