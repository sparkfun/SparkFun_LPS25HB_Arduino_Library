#include "LPS25HB.h"


LPS25HB::LPS25HB( void )
{

}

LPS25HB_CodesTypeDef LPS25HB::begin( TwoWire &wirePort, uint8_t address )
{
	LPS25HB_CodesTypeDef retval = LPS25HB_CODE_ERR;

	sensor_address = address;

	_i2cPort = &wirePort;
	_i2cPort->begin();

	retval = isConnected();
	if(retval == LPS25HB_CODE_CONNECTED)
	{
		if(getID() != LPS25HB_DEVID)
		{
			retval = LPS25_HB_CODE_WRONG_ID;
			goto LPS25HB_begin_EXIT;
		}

		// Now setup default values
		uint8_t values[5];

		values[0] = LPS25HB_RES_CONF_DEFAULT;									// Set the resolution configuration to default
		write( LPS25HB_REG_RES_CONF, values, 1, sensor_address );

		values[0] = LPS25HB_CTRL_REG1_PD_ACTIVE | LPS25HB_CTRL_REG1_ODR_25HZ ;	// Turn the sensor ON and set output rate to 25 Hz
		values[1] = LPS25HB_CTRL_REG2_DEFAULT;									// Default
		values[2] = LPS25HB_CTRL_REG3_INT_L | LPS25HB_CTRL_REG3_OD ;			// Set interrupts to output LOW and Open Drain function	
		values[3] = LPS25HB_CTRL_REG4_DEFAULT;									// Default
		values[4] = LPS25HB_INTERRUPT_CFG_DEFFAULT;								// Set the Interrupt CFG register to default
		retval = write( LPS25HB_REG_CTRL_REG1, values, 5, sensor_address);			// Write the values consecutively to the device
	}

	LPS25HB_begin_EXIT:
	lastCode = retval;
	return retval;		// Notify that the device is disconnected
}





LPS25HB_CodesTypeDef LPS25HB::isConnected()
{
	LPS25HB_CodesTypeDef retval = LPS25HB_CODE_DISCONNECTED;

	_i2cPort->beginTransmission(sensor_address);						// Check the desired address
  	if(_i2cPort->endTransmission() == 0)
  	{
  		retval = LPS25HB_CODE_CONNECTED;								// If connected then set code to connected
  	}

  	lastCode = retval;													// Store the code
  	return retval;														// Return value
}

uint8_t LPS25HB::getID()
{
	uint8_t retval = 0x00;											
	lastCode = read( LPS25HB_REG_WHO_AM_I, &retval, 1, sensor_address);	// Perform the read and store the code returned
	return retval;														// Return the read value
}

uint8_t LPS25HB::getStatus()
{
	uint8_t retval = 0x00;
	lastCode = read( LPS25HB_REG_STATUS_REG, &retval, 1, sensor_address );
	return retval;
}














int16_t		LPS25HB::getTemperature_raw()
{
	uint8_t data[2];
	read(LPS25HB_REG_TEMP_OUT_L, data, 2, sensor_address);
	int16_t retval = 0x00;
	// retval = ((data[1] << 8) | (data[0] << 0));
	retval = (data[1] << 8 | data[0]);
	return retval;
}

float		LPS25HB::getTemperature_degC()
{
	int16_t raw = getTemperature_raw();
	return (float)(raw/480.0);
}

int32_t 	LPS25HB::getPressure_raw()
{
	uint8_t data[3];
	read(LPS25HB_REG_PRESS_OUT_XL, data, 3, sensor_address);
	int32_t retval = 0x00;
	retval = ((int32_t)data[0] << 0) | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16);
	if(data[2] & 0x80){retval |= 0xFF000000; } //This formats the 32 bit data type to twos complement
	return retval;
}

float		LPS25HB::getPressure_hPa()
{
	int32_t raw = getPressure_raw();
	return (float)(raw/4096.0);
}





















LPS25HB_CodesTypeDef LPS25HB::setReferencePressure(uint32_t adc_val)
{
	uint8_t data[3];
	data[0] = ((adc_val & 0x0000FF) >> 0);
	data[1] = ((adc_val & 0x00FF00) >> 8);
	data[2] = ((adc_val & 0xFF0000) >> 16);
	write( LPS25HB_REG_REF_P_XL, data, 3, sensor_address);
}

LPS25HB_CodesTypeDef LPS25HB::setPressureThreshold(uint16_t adc_val)
{
	uint8_t data[2];
	data[0] = ((adc_val & 0x00FF) >> 0);
	data[1] = ((adc_val & 0xFF00) >> 8);
	write( LPS25HB_REG_THS_P_L, data, 2, sensor_address);
}

LPS25HB_CodesTypeDef LPS25HB::setTemperatureAverages(uint8_t avg_code)
{
	uint8_t prev_setting;													// Store the old settings
	read(LPS25HB_REG_RES_CONF, &prev_setting, 1, sensor_address);

	uint8_t new_setting = ((prev_setting & 0x03) | (avg_code & 0x0C));		// Mask the old settings and add in the new
	return write(LPS25HB_REG_RES_CONF, &new_setting, 1, sensor_address);	// Write the new combined settings
}

LPS25HB_CodesTypeDef LPS25HB::setPressureAverages(uint8_t avg_code)
{
	uint8_t prev_setting;													// Store the old settings
	read(LPS25HB_REG_RES_CONF, &prev_setting, 1, sensor_address);

	uint8_t new_setting = ((prev_setting & 0x0C) | (avg_code & 0x03));		// Mask the old settings and add in the new
	return write(LPS25HB_REG_RES_CONF, &new_setting, 1, sensor_address);	// Write the new combined settings
}

LPS25HB_CodesTypeDef LPS25HB::setOutputDataRate(uint8_t odr_code)
{
	uint8_t prev_setting;													// Store the old settings
	read(LPS25HB_REG_CTRL_REG1, &prev_setting, 1, sensor_address);

	uint8_t new_setting = ((prev_setting & 0x8F) | (odr_code & 0x70));		// Mask the old settings and add in the new
	return write(LPS25HB_REG_CTRL_REG1, &new_setting, 1, sensor_address);	// Write the new combined settings
}

LPS25HB_CodesTypeDef LPS25HB::setFIFOMode(uint8_t mode_code)
{
	uint8_t prev_setting;
	uint8_t new_setting;
	read(LPS25HB_REG_CTRL_REG2, &prev_setting, 1, sensor_address);

	if(mode_code == LPS25HB_FIFO_CTRL_BYPASS)
	{
		new_setting = (prev_setting & (~LPS25HB_CTRL_REG2_FIFO_EN));			// Clears the FIFO enable bit
		if(write(LPS25HB_REG_CTRL_REG2, &new_setting, 1, sensor_address) != LPS25HB_CODE_NOM){return LPS25HB_CODE_ERR;}
	}
	else
	{
		new_setting = (prev_setting | LPS25HB_CTRL_REG2_FIFO_EN);			// Enables the FIFO
		if(write(LPS25HB_REG_CTRL_REG2, &new_setting, 1, sensor_address) != LPS25HB_CODE_NOM){return LPS25HB_CODE_ERR;}
	}

	read(LPS25HB_REG_FIFO_CTRL, &prev_setting, 1, sensor_address);
	new_setting = ((prev_setting & 0x1F) | (mode_code & 0xE0));
	if(write(LPS25HB_REG_FIFO_CTRL, &new_setting, 1, sensor_address) != LPS25HB_CODE_NOM){return LPS25HB_CODE_ERR;}			// Sets the new FIFO mode
	return LPS25HB_CODE_NOM;
}

LPS25HB_CodesTypeDef LPS25HB::setFIFOMeanNum(uint8_t num_code)
{
	uint8_t prev_setting;
	uint8_t new_setting;

	read(LPS25HB_REG_FIFO_CTRL, &prev_setting, 1, sensor_address);
	new_setting = ((prev_setting & 0xE0) | (num_code & 0x1F));
	if(write(LPS25HB_REG_FIFO_CTRL, &new_setting, 1, sensor_address) != LPS25HB_CODE_NOM){return LPS25HB_CODE_ERR;}
	return LPS25HB_CODE_NOM;
}












LPS25HB_CodesTypeDef LPS25HB::read( uint8_t reg_adr, uint8_t * pdata, uint8_t size, uint8_t address = LPS25HB_I2C_ADDR_DEF)
{
	LPS25HB_CodesTypeDef retval = LPS25HB_CODE_ERR;

	_i2cPort->beginTransmission( address );				// Begin talking to the desired sensor
	_i2cPort->write(reg_adr | (1<<7));

	if (_i2cPort->endTransmission(false) != 0) 			//Send a restart command. Do not release bus.
  	{
    	retval = LPS25HB_CODE_ERR; 						//Sensor did not ACK
    	goto LPS25HB_read_EXIT;
  	}

  	_i2cPort->requestFrom(address, size);				// Default value of true used here to release bus once complete  	
  	for(uint8_t indi = 0; indi < size; indi++)
  	{
  		if(_i2cPort->available())
  		{
  			*(pdata + indi) = _i2cPort->read();
  		}
  		else
  		{
  			retval = LPS25HB_CODE_RX_UNDERFLOW; 						//Sensor did not ACK
    		goto LPS25HB_read_EXIT;
  		}
  	}		
  	retval = LPS25HB_CODE_NOM; 						//Sensor did ACK

  	LPS25HB_read_EXIT:  	
	lastCode = retval;
	return retval;								
}

LPS25HB_CodesTypeDef LPS25HB::write( uint8_t reg_adr, uint8_t * pdata, uint8_t size, uint8_t address = LPS25HB_I2C_ADDR_DEF)
{
	LPS25HB_CodesTypeDef retval = LPS25HB_CODE_ERR;

	_i2cPort->beginTransmission( address );				// Begin talking to the desired sensor
	_i2cPort->write(reg_adr | (1<<7));					// Specify a write to the desried register with the mutli-byte bit set
	_i2cPort->write(pdata, size);						// Write all the bytes
	
	if (_i2cPort->endTransmission() == 0)
  	{
    	retval = LPS25HB_CODE_NOM; 						// Sensor did ACK
  	}

  	lastCode = retval;
  	return retval;
}