#include "lm75.h"
#include <soft_i2c.h>

SoftwareI2C Wire1;

void tempBegin()
{
    Wire1.begin();
    lm75aPowerOn();
}

uint16_t lm75aReadData(uint8_t dataAddr)
{
    uint16_t Data = 0; 
    Wire1.beginTransmission(LM75A_I2C_ADDR);
    Wire1.write(dataAddr);
    Wire1.endTransmission();

	switch(dataAddr)
	{ 
		case LM75A_TEMP:
		case LM75A_THYST:
		case LM75A_TOS:

            Wire1.requestFrom(LM75A_I2C_ADDR, 2);
            while(Wire1.available())
            {
                Data = Wire1.read();
                Data <<= 8;
                Data |= Wire1.read();
            }

		break;

		case LM75A_CONF:

            Wire1.requestFrom(LM75A_I2C_ADDR, 1);
            while(Wire1.available())
            {
                Data = Wire1.read();
            }
            
		break;

		default:
		break;
	}

	return Data;
}

void lm75aWriteData(uint8_t dataAddr, uint16_t dataValue)
{
    Wire1.beginTransmission(LM75A_I2C_ADDR);

	switch(dataAddr)
	{
		case LM75A_THYST:
		case LM75A_TOS:
            Wire1.write((dataValue >> 1) & 0xff);
            Wire1.write(((dataValue & 0x01) << 7) & 0xff);
            Wire1.endTransmission();
		break;

		case LM75A_CONF:

            Wire1.write((dataValue) & 0xff);
            Wire1.endTransmission();
		break;

		default:
		break;
	}
}

void lm75aPowerOn(void)
{
	lm75aWriteData(LM75A_CONF,0x00);
}

void lm75aPowerOff(void)
{
	lm75aWriteData(LM75A_CONF,0x01);
}

int16_t lm75aGetTempData(void)
{
	uint16_t data = 0;

	data = lm75aReadData(LM75A_TEMP);
	data >>= 5;
	if(data & 0x0400)data |= 0xf800;
	return ((int16_t)(data)) / 8;
}

// END FILE