// temperature

#include <EEPROM.h>
#include <Wire.h>
#include <soft_i2c.h>

#include "LonganCards.h"
#include "cardBasic.h"
#include "CardsDfs.h"


// update sensor here
void valueInfoSet()
{
    dtaCnt = 1;             // only ONE sensor
    
    dtaType[0] = DTA_TYPE_INT;            // temp
    dtaUnit[0] = UNIT_ANALOG;
}

// get sensor data here

int getAnalog()
{
    long sum = 0;
    for(int i=0; i<32; i++)sum += analogRead(A0);
    return sum>>5;
}

void sensorUpdate()
{
    static unsigned long timer_s = 0;
    if(millis()-timer_s < 10)return;
    timer_s = millis();
    
    valueSensor[0] = 1023-getAnalog();

}

// INIT SENSOR HERE
void sensorInit()
{
    // nothing
}

// CARD INFO SET HERE
void cardInfoInit()
{
    cardInfo.setInfo(0x07,                  // I2C ADDR
                     7,                     // SENSOR NUNBER
                     dtaCnt,                // NO. OF SENSOR
                     12,                    // LENGTH OF NAME
                     "Rotary Angle",        // NAME OF SENSOR
                     1011007,               // SKU
                     1.0);                  // VERSION
}

void setup() 
{
    Serial.begin(115200);
    
    valueInfoSet(); 
    cardInfoInit();
    cardInfo.disp();
    cardInit();
    sensorInit();
}

void loop() 
{
    sensorUpdate();
    blinkProcess();
    //dispSensor();
}

void dispSensor()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();
    Serial.println((int)valueSensor[0]);
}

// END FILE
