/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>


int value = 0; // set values you need to zero
Servo firstESC, secondESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time
unsigned long start, finished, elapsed = 0;
boolean valveOpen = false;
int height = 0;  // current height of the balloon relative to the ground
int minSpeed = 70;
int maxSpeed = 500;
int offset = 30;
boolean direction = false;


void setup() {
  // SETUP MOTORS
  firstESC.attach(9);    // attached to pin 9 with motor 1
  secondESC.attach(10);  // attached to pin 10 with motor 2
  Serial.begin(115200);    // start serial at 9600 baud
  Serial.println("0=minThrottle, 1023=maxThrottle");
  value = 1023;
  firstESC.write(1023);
  secondESC.write(1023);
 
  // SETUP VALVE
  // blink light
  //pinMode(13,OUTPUT);
  //digitalWrite(13,LOW);
  
  //polarity switch
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
 
 
  //70-540 work as valid values
  //0 is break
}

void idle() {
   Serial.println("Idle Mode"); 
}

void test() {
  
    Serial.println("Entering Testing Mode ...");
    Serial.println("Step back!");
    delay(3000);
    firstESC.write(100);  // tests fan blades
    secondESC.write(100);  // tests fan blades
    delay(9000);
    Serial.println("Exiting Test Mode..");
    delay(3000);
    value = 0;  // Brings the program back to Idle Mode
}


/*
void launch() {
  while (height != 0) {
    
    if (height < 0.02) {  // stops motors if height is less than 70m altitude
      firstESC.write(0);
      secondESC.write(0);
    }
    else {  // runs when drone is 70m+ altitude
      fixOrientation(); 
      fixDisplacement();
    } 
  }
}
*/

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
    ///digitalWrite(11,LOW);
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
    //digitalWrite(12,LOW);
    Serial.println("valve is now open");
  }
}

/*
void fixOrientation() {
  // Fixes orientation if the drone points off the original launch direction by an offset of 35 degrees
  
  //
  while (currentBearing < offset - desiredBearing) {
    // turn motor 1 on (left), motor 2 off (right)
    firstESC.write(540);  
    secondESC.write(0); 
  }
  
  while (currentBearing > offset + desiredBearing) {
    firstESC.write(0);  // tests fan blades
    secondESC.write(540);  // tests fan blades 
  }
  // if the drone is between both offsets, turn off the blades  
  // firstESC.write(0);  // tests fan blades
  // secondESC.write(0);  // tests fan blades 
}
*/

/*
void checkOrientation() {
  if (offset - desiredBearing <= currentBearing <= offset + desiredBearing) {
    direction = true;
  }
  else {
    direction = false;
  } 
}
*/

/*
void fixDisplacement() {
  // Fixes the displacement of the drone if it's off by a displacement of 2km
  while (displacement > 2 && direction == true) {  // displacement is greater than 10km
    firstESC.write(540);  
    secondESC.write(540);
    checkOrientation();
  }
  // exits method if the direction of the drone is no longer true
}
*/

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  firstESC.write(value);
  secondESC.write(value);
  Serial.println(value);
 
  if(Serial.available()) 
  {
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value);  //prints value motor set to
    
    if (value == 0) {
      idle();
    }
    else if (value == 1) {
      test();
    }
    else if (value == 2) {
      if (valveOpen == true) {
        valveOpen = false;
        value = 0; 
      }
      else {
      valveOpen = true;
      value = 0;
    } 
      valveSwitch();
    }
    else if (value == 3) {
      //launch();
    }
    else {  // random values entered will keep the program in Idle Mode
      value = 0;
    }
  }
  delay(2000);
}
