int sensorPin_Low = A0;    // select the input pin for the potentiometer
int sensorPin_High = A1;    // select the input pin for the potentiometer
//int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
uint16_t maxSoundData = 0;
uint16_t minSoundData = 1023;
uint16_t curSoundData = 0;
uint32_t samplePreviousMillis = 0;
#define SOUND_SAMPPLE_TIME  50

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
    uint16_t sampleData = analogRead(sensorPin_High);
    if(maxSoundData < sampleData)maxSoundData = sampleData;
    else if(minSoundData > sampleData)minSoundData = sampleData;
    
    uint32_t sampleCurrentMillis = millis(); 
    if(sampleCurrentMillis - samplePreviousMillis >= SOUND_SAMPPLE_TIME)
    {
        samplePreviousMillis = sampleCurrentMillis;
        
        curSoundData = maxSoundData - minSoundData;
        maxSoundData = 0;
        minSoundData = 1023;
        
        Serial.println(curSoundData);
    }
}
