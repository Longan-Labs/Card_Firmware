// temperature

#include <EEPROM.h>
#include <Wire.h>
#include <soft_i2c.h>

#include "LonganCards.h"
#include "cardBasic.h"
#include "CardsDfs.h"


#include "lm75.h"


#define SENSOR_UPDATE_TIME  10      // ms

unsigned char nFloat[10];

// update sensor here
void valueInfoSet()
{
    dtaCnt = 1;             // only ONE sensor
    
    dtaType[0] = DTA_TYPE_INT;            // temp
    dtaUnit[0] = UNIT_TEMPERATURE;
}

// get sensor data here
void sensorUpdate()
{
    static unsigned long timer_s = 0;
    if(millis()-timer_s < 10)return;
    timer_s = millis();
    
    valueSensor[0] = lm75aGetTempData();

}

// INIT SENSOR HERE
void sensorInit()
{
    tempBegin();                            // init temperature sensor
}

// CARD INFO SET HERE
void cardInfoInit()
{
    cardInfo.setInfo(0x01,                  // I2C ADDR
                     0x01,                  // SENSOR NUNBER
                     dtaCnt,                // NO. OF SENSOR
                     11,                    // LENGTH OF NAME
                     "Temperature",         // NAME OF SENSOR
                     1011001,               // SKU
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
    dispSensor();
}

void dispSensor()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();
    Serial.println((int)valueSensor[0]);
}

// END FILE
