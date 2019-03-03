/***********************************************************************************
 *
 * demo of RTC DS1307
 * display time to serial terminal
 *
 *
 *  time struct define as below:
 *
 *  typedef struct __time{
 *
 *      union
 *      {
 *          uint8 data[7];
 *          
 *          struct
 *          {
 *              uint8 second;
 *              uint8 minute;
 *              uint8 hour;
 *              uint8 week;
 *              uint8 day;
 *              uint8 month;
 *              uint8 year;                     // 2013 -> 13        
 *          };
 *      };
 *
 *  } r_time;
 *
 *
 ***********************************************************************************/
 
#include <Wire.h>
#include <SoftwareSerial.h>

#include "Suli.h"
#include "RTC_DS1307_Suli.h"
#include "RTC_DS1307_Arduino.h"

RTC rtc;


const char str_week[][5] = {
"MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN",
};

// get time from rtc
void getTime(struct __time * t)
{
    rtc.get(t);
}

// display time to serial monitor
void dispTime()
{
    r_time t1;
    getTime(&t1);
    
    Serial.print("20");
    Serial.print(t1.year);
    Serial.print("/");
    Serial.print(t1.month);
    Serial.print("/");
    Serial.print(t1.day);
    Serial.print(" ");
    Serial.print(str_week[t1.week-1]);
    Serial.print(" ");
    Serial.print(t1.hour);
    Serial.print(":");
    Serial.print(t1.minute);
    Serial.print(":");
    Serial.println(t1.second);

}

void setup()
{
    Serial.begin(115200);
    
    rtc.begin();                // initialize rtc
    
}


void loop()
{
    dispTime();                 // display time
    delay(1000);
}

