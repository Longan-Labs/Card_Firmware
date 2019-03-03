// FIRMWARE FOR CARD MODULE BASIC

const int pinLed = 2;


void setup()
{
    Serial.begin(115200);
    pinMode(pinLed, OUTPUT);
}

void blinkblink()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 100)return;
    timer_s = millis();
    
    digitalWrite(pinLed, 1-digitalRead(pinLed));
    Serial.println("card core");
}

void loop()
{
    blinkblink();
    
    while(Serial.available())
    {
        Serial.write(Serial.read());
    }
}
