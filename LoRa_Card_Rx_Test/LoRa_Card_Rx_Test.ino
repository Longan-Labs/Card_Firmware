void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Begin...");
}

void loop()
{
  AT_bypass();
}
void AT_bypass(){
  while(Serial.available()){
      Serial1.print(Serial.read());
  }
  while(Serial1.available()){
    Serial.print(Serial1.read());
  }
}

