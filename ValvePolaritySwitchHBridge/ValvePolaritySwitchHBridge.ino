boolean valveOpen = false;

void valveSwitch();

void setup()
{
  Serial.begin(19200);
  
  //blinking light
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  //HBridge on/off
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  //polarity switch
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
}

void loop()
{
  valveSwitch();
  delay(10000);
}

/**
opens or closes the valve based on its current position
*/
void valveSwitch()
{
  if(valveOpen == true)
  {
    //light off
    digitalWrite(13,LOW);
    
    valveOpen = false;
    //red wire
    digitalWrite(12,LOW);
    //black wire
    digitalWrite(11,HIGH);
    delay(500);
    digitalWrite(11,LOW);
    Serial.println("valve is now closed");
  }
  else
  {
    //light on
    digitalWrite(13,HIGH);
    
    valveOpen = true;
    //black wire
    digitalWrite(11,LOW);
    //red wire
    digitalWrite(12,HIGH);
    delay(500);
    digitalWrite(12,LOW);
    Serial.println("valve is now open");
  }
}
