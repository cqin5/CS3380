

boolean valveOpen = false;
int value = 0;

void valveSwitch();

void setup()
{
  Serial.begin(19200);
  
  //blinking light
  //pinMode(13,OUTPUT);
  //digitalWrite(13,LOW);
  //HBridge on/off
  //pinMode(10,OUTPUT);
  //digitalWrite(10,HIGH);
  //polarity switch
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
}

void loop()
{  
   if(Serial.available()) 
  {
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value);  //prints value motor set to
    
    if (value == 1) {
      valveOpen = true;
      valveSwitch();
    }
    else {  // random values entered will keep the program in Idle Mode
      valveOpen = false;
      valveSwitch();
    }
  }
}

/**
opens or closes the valve based on its current position
*/
void valveSwitch()
{
  if(valveOpen == false)
  {
    //light off
    //digitalWrite(13,LOW);
    
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
    //digitalWrite(13,HIGH);
    
    //black wire
    digitalWrite(11,LOW);
    //red wire
    digitalWrite(12,HIGH);
    delay(500);
    digitalWrite(12,LOW);
    Serial.println("valve is now open");
  }
}
