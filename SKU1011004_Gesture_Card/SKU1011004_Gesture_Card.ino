// temperature

#include <EEPROM.h>
#include <Wire.h>
#include <soft_i2c.h>

#include "LonganCards.h"
#include "cardBasic.h"
#include "CardsDfs.h"

#include <soft_i2c.h>

#include "paj7620.h"

//VL53L1X distanceSensor;


#define GES_REACTION_TIME		500				// You can adjust the reaction time according to the actual circumstance.
#define GES_ENTRY_TIME			800				// When you want to recognize the Forward/Backward gestures, your gestures' reaction time must less than GES_ENTRY_TIME(0.8s). 
#define GES_QUIT_TIME			1000

unsigned long newDtaCnt = 0;
/*
extern unsigned char dtaType[10];
extern unsigned char dtaUnit[10];
extern unsigned char dtaCnt;
*/
// update sensor here
void valueInfoSet()
{
    dtaCnt = 1;             // only ONE sensor
    
    valueSensor[0] = 0;
    dtaType[0] = DTA_TYPE_INT;              // int
    dtaUnit[0] = UNIT_GESTURE;              // gesture
}

// get sensor data here
/*
// GESTURE
#define GESTURE_UP              1
#define GESTURE_DOWN            2
#define GESTURE_LEFT            3
#define GESTURE_RIGHT           4
#define GESTURE_FORWARD         5
#define GESTURE_BACKWARD        6
#define GESTURE_CLOCKWISE       7
#define GESTURE_ANTICLOCK       8
#define GESTURE_WAVE            9
*/
void sensorUpdate()
{
    static unsigned long timer_s = 0;
    if(millis()-timer_s < 50)return;
    timer_s = millis();
    
    valueSensor[0] = millis()/1000;
    
    /*
    uint8_t data = 0, data1 = 0, error;
	
	error = paj7620ReadReg(0x43, 1, &data);				// Read Bank_0_Reg_0x43/0x44 for gesture result.
	if (!error) 
	{
        
		switch (data) 									// When different gestures be detected, the variable 'data' will be set to different values by paj7620ReadReg(0x43, 1, &data).
		{
			case GES_RIGHT_FLAG:
				delay(GES_ENTRY_TIME);
				paj7620ReadReg(0x43, 1, &data);
				if(data == GES_FORWARD_FLAG) 
				{
					Serial.println("Forward");
                    valueSensor[0] = GESTURE_FORWARD;
					delay(GES_QUIT_TIME);
				}
				else if(data == GES_BACKWARD_FLAG) 
				{
					Serial.println("Backward");
                    valueSensor[0] = GESTURE_BACKWARD;
					delay(GES_QUIT_TIME);
				}
				else
				{
					Serial.println("Right");
                    valueSensor[0] = GESTURE_LEFT;
				}          
				break;
                
			case GES_LEFT_FLAG: 
				delay(GES_ENTRY_TIME);
				paj7620ReadReg(0x43, 1, &data);
				if(data == GES_FORWARD_FLAG) 
				{
					Serial.println("Forward");
                    valueSensor[0] = GESTURE_FORWARD;
					delay(GES_QUIT_TIME);
				}
				else if(data == GES_BACKWARD_FLAG) 
				{
					Serial.println("Backward");
                    valueSensor[0] = GESTURE_BACKWARD;
					delay(GES_QUIT_TIME);
				}
				else
				{
					Serial.println("Left");
                    valueSensor[0] = GESTURE_RIGHT;
				}          
				break;
			case GES_UP_FLAG:
				delay(GES_ENTRY_TIME);
				paj7620ReadReg(0x43, 1, &data);
				if(data == GES_FORWARD_FLAG) 
				{
					Serial.println("Forward");
                    valueSensor[0] = GESTURE_FORWARD;
					delay(GES_QUIT_TIME);
				}
				else if(data == GES_BACKWARD_FLAG) 
				{
					Serial.println("Backward");
                    valueSensor[0] = GESTURE_BACKWARD;
					delay(GES_QUIT_TIME);
				}
				else
				{
					Serial.println("Up");
                    valueSensor[0] = GESTURE_DOWN;
				}          
				break;
			case GES_DOWN_FLAG:
				delay(GES_ENTRY_TIME);
				paj7620ReadReg(0x43, 1, &data);
				if(data == GES_FORWARD_FLAG) 
				{
					Serial.println("Forward");
                    valueSensor[0] = GESTURE_FORWARD;
					delay(GES_QUIT_TIME);
				}
				else if(data == GES_BACKWARD_FLAG) 
				{
					Serial.println("Backward");
                    valueSensor[0] = GESTURE_BACKWARD;
					delay(GES_QUIT_TIME);
				}
				else
				{
					Serial.println("Down");
                    valueSensor[0] = GESTURE_UP;
				}          
				break;
			case GES_FORWARD_FLAG:
				Serial.println("Forward");
                valueSensor[0] = GESTURE_FORWARD;
				delay(GES_QUIT_TIME);
				break;
			case GES_BACKWARD_FLAG:		  
				Serial.println("Backward");
                valueSensor[0] = GESTURE_BACKWARD;
				delay(GES_QUIT_TIME);
				break;
			case GES_CLOCKWISE_FLAG:
				Serial.println("Clockwise");
                valueSensor[0] = GESTURE_CLOCKWISE;
				break;
			case GES_COUNT_CLOCKWISE_FLAG:
				Serial.println("anti-clockwise");
                valueSensor[0] = GESTURE_ANTICLOCK;
				break;  
			default:
				paj7620ReadReg(0x44, 1, &data1);
				if (data1 == GES_WAVE_FLAG) 
				{
					Serial.println("wave");
                    valueSensor[0] = GESTURE_WAVE;
				}
				break;
		}
        
        newDtaCnt = millis();
	}*/
}

// INIT SENSOR HERE
void sensorInit()
{
    unsigned char error = paj7620Init();			// initialize Paj7620 registers
	if (error) 
	{
		Serial.print("INIT ERROR,CODE:");
		Serial.println(error);
	}
	else
	{
		Serial.println("INIT OK");
	}
}

// CARD INFO SET HERE
void cardInfoInit()
{
    cardInfo.setInfo(0x04,                  // I2C ADDR
                     4,                     // SENSOR NUNBER
                     dtaCnt,                // NO. OF SENSOR
                     7,                     // LENGTH OF NAME
                     "Gesture",             // NAME OF SENSOR
                     1011004,               // SKU
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
    clearDta();
}

void dispSensor()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();
    Serial.println((int)valueSensor[0]);
}

void clearDta()
{

    if(millis()-newDtaCnt < 1000)return;
    newDtaCnt = millis();
    
    valueSensor[0] = 0;
}

// END FILE
