// BME 280
// 

#include <EEPROM.h>
#include <Wire.h>
#include <soft_i2c.h>

#include "LonganCards.h"
#include "cardBasic.h"
#include "CardsDfs.h"

/*---1-START----------------------------------------------------------
 -
 - Add the library to read the sensor here
 - Add the class object here if need
 -
 --------------------------------------------------------------------*/
#include "Seeed_BME280.h"                                                                                                    
BME280 bme280;                                                      
/*---1-END-----------------------------------------------------------*/


/*---2-START----------------------------------------------------------
 - 
 - SENSOR UPDATE TIME, DEFAULT TO 10ms
 -
 --------------------------------------------------------------------*/
#define SENSOR_UPDATE_TIME  10      // ms
/*---2-END-----------------------------------------------------------*/


unsigned char nFloat[10];


/*---3-START----------------------------------------------------------
 - 
 - SET THE SENSOR INFOS
 -
 --------------------------------------------------------------------*/
void valueInfoSet()
{
    dtaCnt = 4;                
    
    dtaType[0] = DTA_TYPE_FLOAT;                // temp
    dtaUnit[0] = UNIT_TEMPERATURE;
    nFloat[0]  = 0;
    
    dtaType[1] = DTA_TYPE_INT;                  // humidity
    dtaUnit[1] = UNIT_HUMIDITY;
    nFloat[1]  = 0;
    
    dtaType[2] = DTA_TYPE_FLOAT;                // press, kpa
    dtaUnit[2] = UNIT_PRESS;
    nFloat[2]  = 2;
    
    dtaType[3] = DTA_TYPE_FLOAT;                // attidude, meter
    dtaUnit[3] = UNIT_ALTITUDE;
    nFloat[3]  = 2;
}
/*---3-END-----------------------------------------------------------*/



/*---4-START----------------------------------------------------------
 - 
 - INITIALIZE THE SENSOR
 -
 --------------------------------------------------------------------*/
void sensorInit()
{
    if(!bme280.init()){
        Serial.println("Device error!");
    }
}
/*---4-END-----------------------------------------------------------*/


/*---5-START----------------------------------------------------------
 - 
 - SET CARD INFOS
 -
 --------------------------------------------------------------------*/
void cardInfoInit()
{
    cardInfo.setInfo(0x08,                  // I2C ADDR
                     0x05,                  // NO. of Card system
                     dtaCnt,                // Quantity of Sensors
                     6,                     // LENGTH OF NAME
                     "BME280",              // NAME OF SENSOR, max 20 char
                     1011008,               // SKU
                     1.0);                  // VERSION
}
/*---5-END-----------------------------------------------------------*/


/*---6-START----------------------------------------------------------
 - 
 - UPDATE SENSOR VALUE 
 -
 --------------------------------------------------------------------*/
void sensorUpdate()
{
    static unsigned long timer_s = 0;
    if(millis()-timer_s < SENSOR_UPDATE_TIME)return;
    timer_s = millis();
    
    valueSensor[0] = bme280.getTemperature();
    valueSensor[1] = bme280.getHumidity();
    valueSensor[2] = bme280.getPressure()/1000.0;
    valueSensor[3] = bme280.calcAltitude(valueSensor[2]*1000.0);
}
/*---6-END-----------------------------------------------------------*/


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
    
    for(int i=0; i<cardInfo.senCnt; i++)
    {
        Serial.print(valueSensor[i], nFloat[i]);
        Serial.print('\t');
    }
    
    Serial.println();
}

// END FILE