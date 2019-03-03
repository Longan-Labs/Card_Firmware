// temperature

#include <EEPROM.h>
#include <Wire.h>
#include <soft_i2c.h>

#include "LonganCards.h"
#include "cardBasic.h"
#include "CardsDfs.h"

#include <soft_i2c.h>

#include "SparkFun_VL53L1X_Arduino_Library.h"

VL53L1X distanceSensor;


/*
extern unsigned char dtaType[10];
extern unsigned char dtaUnit[10];
extern unsigned char dtaCnt;
*/
// update sensor here
void valueInfoSet()
{
    dtaCnt = 1;             // only ONE sensor
    
    dtaType[0] = DTA_TYPE_INT;              // int
    dtaUnit[0] = UNIT_MILLIMETER;           // mm
}

// get sensor data here
void sensorUpdate()
{
    static unsigned long timer_s = 0;
    if(millis()-timer_s < 10)return;
    timer_s = millis();
    
    if(distanceSensor.newDataReady() == false)return;
    valueSensor[0] = distanceSensor.getDistance();
}

// INIT SENSOR HERE
void sensorInit()
{
    if (distanceSensor.begin() == false)
        Serial.println("Sensor offline!");
}

// CARD INFO SET HERE
void cardInfoInit()
{
    cardInfo.setInfo(0x09,                  // I2C ADDR
                     9,                // SENSOR NUNBER
                     dtaCnt,                  // NO. OF SENSOR
                     8,                    // LENGTH OF NAME
                     "Distance",         // NAME OF SENSOR
                     1011009,               // SKU
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
