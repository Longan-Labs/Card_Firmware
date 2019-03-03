// i2c scan
#include "soft_i2c.h"

SoftwareI2C Wire1;

void setup()
{
    Serial.begin(115200);
    Wire1.begin();       // sda, scl
    Serial.println("begn to scan...");
}

void loop()
{
    for(unsigned char i=1; i<=127; i++)
    {
        if(Wire1.beginTransmission(i))
        {
            Serial.print("0x");
            Serial.println(i, HEX);
            
            while(1);
        }
        Wire1.endTransmission();
    }
    
    Serial.println("find nothing");
    while(1);
    
}
