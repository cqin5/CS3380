 //70-540 work as valid values
  //0 is break

#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <cmath.h>
#include <stdio.h>

#define ACCELE_SCALE 2  // accelerometer full-scale, should be 2, 4, or 8

/* LSM303 Address definitions */
#define LSM303_MAG  0x1E  // assuming SA0 grounded
#define LSM303_ACC  0x18  // assuming SA0 grounded

#define X 0
#define Y 1
#define Z 2

/* LSM303 Register definitions */
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define HP_FILTER_RESET_A 0x25
#define REFERENCE_A 0x26
#define STATUS_REG_A 0x27
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define INT1_CFG_A 0x30
#define INT1_SOURCE_A 0x31
#define INT1_THS_A 0x32
#define INT1_DURATION_A 0x33
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01//refer to the Table 58 of the datasheet of LSM303DLM
	#define MAG_SCALE_1_3 0x20//full-scale is +/-1.3Gauss
	#define MAG_SCALE_1_9 0x40//+/-1.9Gauss
	#define MAG_SCALE_2_5 0x60//+/-2.5Gauss
	#define MAG_SCALE_4_0 0x80//+/-4.0Gauss
	#define MAG_SCALE_4_7 0xa0//+/-4.7Gauss
	#define MAG_SCALE_5_6 0xc0//+/-5.6Gauss
	#define MAG_SCALE_8_1 0xe0//+/-8.1Gauss
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Y_H_M 0x07
#define OUT_Y_L_M 0x08
#define OUT_Z_H_M 0x05
#define OUT_Z_L_M 0x06
#define SR_REG_M 0x09
#define IRA_REG_M 0x0A
#define IRB_REG_M 0x0B
#define IRC_REG_M 0x0C
#define PI 3.14159    // 3.14159

/* Global variables */
int value = 0; // set values you need to zero
Servo firstESC, secondESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time
unsigned long start, finished, elapsed = 0;
boolean valveOpen = false;
int height = 0;  // current height of the balloon relative to the ground
int minSpeed = 70;
int maxSpeed = 500;
int offset = 30;
boolean direction = false;
int flightDuration = 0;

// compass variables
int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float desiredBearing = 0.00;
float currentBearing = 0.00;


void setup() {
  // SETUP MOTORS
  firstESC.attach(9);    // attached to pin 9 with motor 1
  secondESC.attach(10);  // attached to pin 10 with motor 2
  Serial.begin(115200);    // start serial at 9600 baud
  Serial.println("0=minThrottle, 1023=maxThrottle");
  value = 1023;
  firstESC.write(1023);
  secondESC.write(1023);

  // valve code
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
  
  // Latitude and longitude
  // latA & lonA is the current coordinates
  // latB and lonB is the coordinates of where the balloon is expected to land
  float latA = 0;
  float lonA = 0;
  
  float latB = 43.015277;
  float lonB = -81.280354;
  
  // Compass code
   Wire.begin();  // Start up I2C, required for LSM303 communication
  initLSM303(ACCELE_SCALE);  // Initialize the LSM303, using a SCALE full-scale range 
}


void initLSM303(int fs)
{
  LSM303_write(0x27, CTRL_REG1_A);  // 0x27 = normal power mode, all accel axes on
  if ((fs==8)||(fs==4))
    LSM303_write((0x00 | (fs-fs/2-1)<<4), CTRL_REG4_A);  // set full-scale
  else
    LSM303_write(0x00, CTRL_REG4_A);
  LSM303_write(0x14, CRA_REG_M);  // 0x14 = mag 30Hz output rate
  LSM303_write(MAG_SCALE_1_3, CRB_REG_M); //magnetic scale = +/-1.3Gauss
  LSM303_write(0x00, MR_REG_M);  // 0x00 = continouous conversion mode
}

void printValues(int * magArray, int * accelArray)
{
  /* print out mag and accel arrays all pretty-like */
  Serial.print(accelArray[X], DEC);
  
  Serial.print("\t");
  Serial.print(accelArray[Y], DEC);
  Serial.print("\t");
  Serial.print(accelArray[Z], DEC);
  Serial.print("\t\t");
  
  Serial.print(magArray[X], DEC);
  Serial.print("\t");
  Serial.print(magArray[Y], DEC);
  Serial.print("\t");
  Serial.print(magArray[Z], DEC);
  Serial.println();
}
float getHeading(int * magValue)
{
  // see section 1.2 in app note AN3192
  float heading = 180*atan2(magValue[Y], magValue[X])/PI;  // assume pitch, roll are 0
  
  if (heading <0)
    heading += 360;
  
  return heading;
}


float getTiltHeading(int * magValue, float * accelValue)
{
  // see appendix A in app note AN3192 
  float pitch = asin(-accelValue[X]);
  float roll = asin(accelValue[Y]/cos(pitch));
  
  float xh = magValue[X] * cos(pitch) + magValue[Z] * sin(pitch);
  float yh = magValue[X] * sin(roll) * sin(pitch) + magValue[Y] * cos(roll) - magValue[Z] * sin(roll) * cos(pitch);
  float zh = -magValue[X] * cos(roll) * sin(pitch) + magValue[Y] * sin(roll) + magValue[Z] * cos(roll) * cos(pitch);
  float heading = 180 * atan2(yh, xh)/PI;

  if (yh >= 0)    return heading;  
  else    return (360 + heading);
}

void getLSM303_mag(int * rawValues)
{
  Wire.beginTransmission(LSM303_MAG);
  Wire.write(OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM303_MAG, 6);
  for (int i=0; i<3; i++)
    rawValues[i] = (Wire.read() << 8) | Wire.read();
  int temp;
  temp = rawValues[Y];
  rawValues[Y] = rawValues[Z];
  rawValues[Z] = temp;  
}

void getLSM303_accel(int * rawValues)
{
  rawValues[Z] = ((int)LSM303_read(OUT_X_L_A) << 8) | (LSM303_read(OUT_X_H_A));
  rawValues[X] = ((int)LSM303_read(OUT_Y_L_A) << 8) | (LSM303_read(OUT_Y_H_A));
  rawValues[Y] = ((int)LSM303_read(OUT_Z_L_A) << 8) | (LSM303_read(OUT_Z_H_A));
  // had to swap those to right the data with the proper axis
}

byte LSM303_read(byte address)
{
  byte temp;
  
  if (address >= 0x20)
    Wire.beginTransmission(LSM303_ACC);
  else
    Wire.beginTransmission(LSM303_MAG);
    
  Wire.write(address);
  
  if (address >= 0x20)
    Wire.requestFrom(LSM303_ACC, 1);
  else
    Wire.requestFrom(LSM303_MAG, 1);
  while(!Wire.available())
    ;
  temp = Wire.read();
  Wire.endTransmission();
  
  return temp;
}

void LSM303_write(byte data, byte address)
{
  if (address >= 0x20)
    Wire.beginTransmission(LSM303_ACC);
  else
    Wire.beginTransmission(LSM303_MAG);
    
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}


void idle() {
   Serial.println("Idle Mode"); 
}

void test() {  
    Serial.println("Entering Testing Mode ...");
    Serial.println("Step back!");
    delay(9000);
    firstESC.write(70);  // tests fan blades
    secondESC.write(70);  // tests fan blades
    delay(180000);
    Serial.println("Exiting Test Mode..");
    delay(3000);
    value = 0;  // Brings the program back to Idle Mode
}

/*
void land() {
 // open the valve
  valveSwitch();
 
  
}
*/

void getLocation() {
  Wire.requestFrom(2,9);    // request 6 bytes from slave device #2

  /*
  while(Wire.available())    // slave may send less than requested
  { 
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character   
  } 
  */
  
  char c = Wire.read();
  
  if ( 
  
}


void launch() {
  while (height != 0) {
    
    if (height < 0.02) {  // stops motors if height is less than 70m altitude
      firstESC.write(0);
      secondESC.write(0);
    }
    else if (flightDuration > 7200 && altitude < 18000) {  // lands if below 18km
      land();
    }
    else {  // runs when drone is 70m+ altitude
      fly(); 
    } 
  }
}



// opens or closes the valve based on its current position

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
    //digitalWrite(11,LOW);
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
   // digitalWrite(12,LOW);
    Serial.println("valve is now open");
  }
}


void getBearing() {
  /* This is the formula to calculate the desired bearing based on current position and desired position
   Δφ = ln( tan( latB / 2 + π / 4 ) / tan( latA / 2 + π / 4) ) 
  Δlon = abs( lonA - lonB ) 
  bearing :  θ = atan2( Δlon ,  Δφ ) 

  Note: 1) ln = natural log      2) if Δlon > 180°  then   Δlon = Δlon (mod 180).
 */ 
 deltaPi = log( tan( latB / 2 + PI /4 ) / tan( latA / 2 + PI / 4 ));
 deltaLon = abs( lonA - lonB );
 
 if ( deltaLon > 180 ) {
   deltaLon = deltaLon % 180;
 }
  
  desiredBearing = atan2( deltaLon , deltaPi );
}



void fly() {
  // Gets the desired bearing
  getBearing();
  
  // Fixes orientation if the drone points off the original launch direction by an offset of 35 degrees
  while (currentBearing < offset - desiredBearing) {
    // turn motor 1 on (left), motor 2 off (right)
    firstESC.write(540);  
    secondESC.write(0); 
  }
  
  while (currentBearing > offset + desiredBearing) {
    firstESC.write(0);  // tests fan blades
    secondESC.write(540);  // tests fan blades 
  }
  
  while (offset - desiredBearing < currentBearing < offset + desiredBearing) {
    // if the drone is between both offsets, turn the blades on  
     firstESC.write(540);  // tests fan blades
     secondESC.write(540);  // tests fan blades 
  }
}



void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  firstESC.write(value);
  secondESC.write(value);
  Serial.println(value);
  delay(1000);
  flightDuration++;
 

 /** Compass code **/
 Serial.println("**************");
  getLSM303_accel(accel);  // get the acceleration values and store them in the accel array
  while(!(LSM303_read(SR_REG_M) & 0x01))
    ;  // wait for the magnetometer readings to be ready
  getLSM303_mag(mag);  // get the magnetometer values, store them in mag
  //printValues(mag, accel);  // print the raw accel and mag values, good debugging
  Serial.println("Acceleration of X,Y,Z is");
  for (int i=0; i<3; i++)
  {
    realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
	Serial.print(realAccel[i]);
	Serial.println("g");
  }
  /* print both the level, and tilt-compensated headings below to compare */
  //Serial.println("The clockwise angle between the magnetic north and x-axis: ");
  //Serial.print(getHeading(mag), 3); // this only works if the sensor is level
  currentBearing = getHeading(mag);
  
  //Serial.println(" degrees");
  //Serial.print("The clockwise angle between the magnetic north and the projection");
  //Serial.println(" of the positive x-axis in the horizontal plane: ");
  //Serial.print(getTiltHeading(mag, realAccel), 3);  // see how awesome tilt compensation is?!
  //Serial.println(" degrees");
  //delay(200);  // delay for serial readability
 
 /** End of compass code **/
 

 
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
}
