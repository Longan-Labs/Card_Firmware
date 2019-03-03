#include <TimerOne.h>


// 近的时候D9 HIGH
// 远的时候D9 LOW

int cnt = 0;

void makePulse()
{
    cnt = 20;
    Timer1.initialize(12); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
    Timer1.attachInterrupt( timerIsr ); // attach the service routine here
    Timer1.start();

}

void setup() 
{
  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  pinMode(13, OUTPUT);    
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(9,LOW);
  
  pinMode(10, INPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  
  
  Serial.begin(9600);
  //Timer1.initialize(12); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  //Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  

   
}

const int ULTCNT = 10;

unsigned long timer_s = millis();
// timeout, 15ms, 5m

int getmm()
{
    Serial.println("START");

    unsigned long timer_s = micros();
    unsigned long timer_e = 0;
    unsigned long timer_t = millis();
    
    makePulse();
    
    delay(2);
   
    int index = 0;
    
    while((millis()-timer_t) < 30)
    {

        if((PINB&0b100) == 0)
        {
            timer_e = micros();
            index++;
            break;
        }
        
    }
    
     if(index)
     {
         unsigned long dt = timer_e - timer_s;
         Serial.print("dt = ");
         Serial.println(dt);
         Serial.print("distance = ");
         Serial.print((float)dt/10000.0*340.0/2.0);
         Serial.println(" cm");
     }
    
    
END:

    Serial.println("END\r\n----------------------------------------");
}

void loop()
{
  // Main code loop
  // TODO: Put your regular (non-ISR) logic here
  
    //cnt = 20;
    //delay(1000);
    
    getmm();
    delay(1000);
    
    //Serial.println(digitalRead(10));
    
    //unsigned long timer_s = millis();
    //while(millis()-timer_s < 1000);
    /*if(millis()-timer_s < 1000)
    {
      cnt = 0;
    }
    else
    {
      cnt = 20;
      timer_s = millis();
    }*/

}
 
/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------

void timerIsr()
{
    if(cnt == 0)return;
    
    cnt--;
    
    if(cnt == 0)
    {
        Timer1.stop();
    }
    
    if(cnt%2)
        PORTD |= 0b00001000;
    else 
        PORTD &= 0b11110111;
}