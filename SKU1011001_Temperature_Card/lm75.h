// LM75 DRIVER
#ifndef __LM75A_H__
#define __LM75A_H__

#include <Arduino.h>

#define LM75A_I2C_ADDR	0x48
#define LM75A_TEMP		0x00
#define LM75A_CONF		0x01
#define LM75A_THYST		0x02
#define LM75A_TOS		0x03
#define LM75A_ID		0x07

void tempBegin();
uint16_t lm75aReadData(uint8_t dataAddr);
void lm75aWriteData(uint8_t dataAddr, uint16_t dataValue);
void lm75aPowerOn(void);
void lm75aPowerOff(void);
int16_t lm75aGetTempData(void);

#endif
// END FILE