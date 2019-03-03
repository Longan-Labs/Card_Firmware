#include <Wire.h>
#include "CardsDfs.h"

#define ADDR    0x08

int cntSensor = 0;

float sensorValue[10];

float str2num(unsigned char *str)
{
    float num = 0;
    memcpy((unsigned char *)(&num), str, 4);
    return num;
}

int getCntSensor()
{
    unsigned char cnt = 0;
    
    Wire.beginTransmission(ADDR);
    Wire.write(ADDR_SNESOR_CNT);    // ADDR_SNESOR_CNT = 0x02
    Wire.endTransmission();
    
    delay(1);
    Wire.requestFrom(ADDR, 1);
    
    delay(1);
    
    while(Wire.available())
    {
        cnt = Wire.read();
    }
    
    return cnt;
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Wire.begin();
    
    while(!Serial.available());
    
    cntSensor = getCntSensor();
    Serial.print("cnt = ");
    Serial.println(cntSensor);
}

void loop()
{
    for(int i=0; i<cntSensor; i++)
    {
        sensorValue[i] = getValue(i);
        Serial.print(sensorValue[i], 2);
        Serial.print('\t');
    }
    
    Serial.println();
    
    delay(100);

}

float getValue(int s)
{
    unsigned char dta[10];
    unsigned long len = 0;
    
    Wire.beginTransmission(ADDR);
    Wire.write(ADDR_DATA_START+s);      // ADDR_DATA_START = 0x30
    Wire.endTransmission();
    
    delay(1);
    Wire.requestFrom(ADDR, 6);
    
    delay(1);
    
    while(Wire.available())
    {
        dta[len++] = Wire.read();
    }
    
    return str2num(&dta[1]);
}