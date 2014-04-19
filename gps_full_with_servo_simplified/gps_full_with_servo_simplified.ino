
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
#include <SPI.h>
//#include <stdlib.h>
//#include <stdio.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>


SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false  

// Set the pins used
#define chipSelect 10
#define ledPin 13

File logfile;

/* Servo init.*/
int value = 0; // set values you need to zero
Servo firstESC, secondESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time
unsigned long start, finished, elapsed = 0;
boolean valveOpen = false;
int height = 0;  // current height of the balloon relative to the ground
int minSpeed = 70;
int maxSpeed = 500;
/* End of servo init */


/* Compass init. */
#define ACCELE_SCALE 2  // accelerometer full-scale, should be 2, 4, or 8

/* LSM303 Address definitions */
#define 0x1E    // assuming SA0 grounded
#define 0x18    // assuming SA0 grounded

#define X 0
#define Y 1
#define Z 2



#define 0x30
#define 0x31
#define 0x32
#define 0x33
#define 0x00
#define 0x01 //refer to the Table 58 of the datasheet of LSM303DLM
	#define 0x20 //full-scale is +/-1.3Gauss
	#define 0x40 //+/-1.9Gauss
	#define 0x60 //+/-2.5Gauss
	#define 0x80 //+/-4.0Gauss
	#define 0xa0 //+/-4.7Gauss
	#define 0xc0 //+/-5.6Gauss
	#define 0xe0 //+/-8.1Gauss
#define 0x02
#define 0x03
#define 0x04
#define 0x07
#define 0x08
#define 0x05
#define 0x06
#define 0x09 
#define 0x0A 
#define 0x0B 
#define 0x0C
/* Compass init. end */



int mag[3];  // raw magnetometer values stored here


// if available ram is less then 300, then card init will fail
int freeRam() 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup() {
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  
  
  Serial.begin(115200);

  //If ram drops below 300 bytes, it wouldnt be able to continue. 
  Serial.print("R:");
  Serial.println(freeRam());

  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect, 11, 12, 13)) {
  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("E1"); // Car init. failed
    error(2);
  }
  
  Serial.println("a");
  
  char filename[2];
  strcpy(filename, "00");
  for (uint8_t i = 0; i < 100; i++) {
    filename[0] = '0' + i/10;
    filename[1] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  //If ram drops below 300 bytes, it wouldnt be able to continue. 
  Serial.print("R:");
  Serial.println(freeRam());
  
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("E2"); // Couldn't write to file
    error(3);
  }
  Serial.print("W:"); Serial.println(filename);
  
  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 or 5 Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);
  
  //Serial.println("Ready!");
  
  
  /*
  Servo
  */
    // SETUP MOTORS
  firstESC.attach(5);    // attached to pin 9 with motor 1
  secondESC.attach(6);  // attached to pin 10 with motor 2
  //Serial.begin(115200);    // start serial at 9600 baud
  //Serial.println("0=minThrottle, 1023=maxThrottle");
  value = 1023;
  firstESC.write(1023);
  secondESC.write(1023);
 
  // SETUP VALVE
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  //HBridge on/off
  pinMode(3,OUTPUT);
  digitalWrite(10,HIGH);
  //polarity switch
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
  
  
  /*
  End of Servo
  */
  
  
  
  // Compass
  Wire.begin();  // Start up I2C, required for LSM303 communication
  initLSM303(ACCELE_SCALE);  // Initialize the LSM303, using a SCALE full-scale range
  
  
}




/**************************************

Servo functions


***************************************/


void idle() {
   Serial.println("M1"); // Idle mode
}

void test() {
  
    Serial.println("M2"); // Entering testing mode
    delay(3000);
    firstESC.write(100);  // tests fan blades
    secondESC.write(100);  // tests fan blades
    delay(1000);
    Serial.println("M2x"); // Exiting testing mode
    delay(3000);
    value = 0;  // Brings the program back to Idle Mode
}

void fill() {
  valveSwitch();
  delay(4000);
}

void launch() {
  Serial.println("M3"); // Preparing to launch
  while (height <= 30) {
     delay(1000);  // junk code
  }
  
  while (height > 30) {  // turns on the motor once it has reached 30m above ground
     fixOrientation(); 
     fixDisplacement();
    }  
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
    Serial.println("V0"); // Valve closed
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
    Serial.println("V1"); // Valve open
  }
}


void fixOrientation() {
  // Fixes orientation if the drone points off the original launch direction by an offset of 35 degrees
  
  
}


void fixDisplacement() {
  // Fixes the displacement of the drone if it's off by a displacement of 20m
  
  
  // exits method if the orientation of the drone is off by 30 degrees
}




/**************************************

End of servo functions


***************************************/

void loop() {
  get_gps_data();
  Serial.print("Lat: "); Serial.println(get_gps_data(1));
  Serial.print("Lon: "); Serial.println(get_gps_data(2));
  get_distance();
  //Serial.print("Ram: ");
  //Serial.println(freeRam());
  //Serial.println();
  //delay(200);
  
  
  //First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  /* Servo code */
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
      fill();
    }
    else if (value == 3) {
      launch();
    }
    else {  // random values entered will keep the program in Idle Mode
      value = 0;
    }
  }
  
  
  
  /* End of Servo code */
  
 
  while(!(LSM303_read(0x09) & 0x01))
    ;  // wait for the magnetometer readings to be ready
  get0x1E(mag);  // get the magnetometer values, store them in mag
  
  Serial.print("A: ");
  Serial.println(getHeading(mag), 3); // this only works if the sensor is level
  
  
  delay(1000);
}




int call_GPS(){

  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return NULL;  // we can fail to parse a sentence in which case we should just wait for another
    
    // Sentence parsed! 
    //Serial.println("OK");
    
    if (LOG_FIXONLY && !GPS.fix) {
        //Serial.print("No Fix");
        return NULL;
    }
  
    // Rad. lets log it!
    //Serial.println("Log");
    
    char *stringptr = GPS.lastNMEA();

    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
      error(4);
    if (strstr(stringptr, "RMC"))   logfile.flush();
    return 1;
  }

  return 0;
}

void get_gps_data(){
  while(call_GPS() == 0);
}


// Unified GPS data parser
// get_latitude         10
// get_lat_hemisphere:  11
// get_longitude        20
// get_lon_hemisphere:  21
// get_altitude:        3
// get_speed:           4
// get_direction:       5
// get_year:            60
// get_month:           61
// get_day:             62
// get_hour:            63
// get_minute:          64
// get_second:          65
// get_satellites:      7

double get_gps_data(int index){
  switch(index) {
    case 10:      return GPS.latitude/100;
    case 11:      if(GPS.lat == 'N') return  1.0;
                  else               return -1.0;
                  
    case 20:      return GPS.longitude/100;
    case 21:      if(GPS.lon == 'E') return  1.0;
                  else               return -1.0;
                  
    case 3:       return GPS.latitude;
    case 4:       return GPS.speed;
    case 5:       return GPS.magvariation;
    
    case 60:      return GPS.year;
    case 61:      return GPS.month;
    case 62:      return GPS.day;
    case 63:      return GPS.hour;
    case 64:      return GPS.minute;
    case 65:      return GPS.seconds;
    
    case 7:       return GPS.satellites;
  }
  return 999.99;
}


// haversine formula
double get_distance(){
  // Richmond St. and Twelve Mile Rd, ~ 12 km from Western
  double latitude_destination  =  43.11;
  double longitude_destination = -81.33;
  
  double latitude_current  = get_gps_data(1);
  double longitude_current = get_gps_data(2) * -1;
  
  if(latitude_current == 0.00 && longitude_current == 0.00){
    Serial.println("E");
    return 0.00;
  }
  else{
    // Print current coordinates after conversion
    //Serial.print("Lat:  ");  Serial.println(latitude_current);
    //Serial.print("Lon: ");   Serial.println(longitude_current);
  
    float dist_calc   = 0;
    float dist_calc2  = 0;
    float diflat      = 0;
    float diflon      = 0;
    
    diflat = radians(latitude_current-latitude_destination);
    latitude_destination = radians(latitude_destination);
    latitude_current = radians(latitude_current);
    diflon = radians((longitude_current)-(longitude_destination));
    
    dist_calc   = (sin(diflat/2.0)*sin(diflat/2.0));
    dist_calc2  = cos(latitude_destination);
    dist_calc2 *= cos(latitude_current);
    dist_calc2 *= sin(diflon/2.0);
    dist_calc2 *= sin(diflon/2.0);
    dist_calc  += dist_calc2;
    
    dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
    
    dist_calc *= 6371000.0; //Converting to meters
    Serial.print("D:");
    Serial.println(dist_calc);
    return dist_calc;
  }
}



/* Compass */
void initLSM303(int fs)
{
  LSM303_write(0x27, 0x20);  // 0x27 = normal power mode, all accel axes on
  if ((fs==8)||(fs==4))
    LSM303_write((0x00 | (fs-fs/2-1)<<4), 0x23);  // set full-scale
  else
    LSM303_write(0x00, 0x23);
  LSM303_write(0x14, 0x00);  // 0x14 = mag 30Hz output rate
  LSM303_write(0x20, 0x01); //magnetic scale = +/-1.3Gauss
  LSM303_write(0x00, 0x02);  // 0x00 = continouous conversion mode
}

/********************************************************************/

float getHeading(int * magValue)
{
  // see section 1.2 in app note AN3192
  float heading = 180*atan2(magValue[Y], magValue[X])/PI;  // assume pitch, roll are 0
  
  if (heading <0)
    heading += 360;
  
  return heading;
}

/*******************************************************************/



void get0x1E(int * rawValues)
{
  Wire.beginTransmission(0x1E);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x1E, 6);
  for (int i=0; i<3; i++)
    rawValues[i] = (Wire.read() << 8) | Wire.read();
  int temp;
  temp = rawValues[Y];
  rawValues[Y] = rawValues[Z];
  rawValues[Z] = temp;  
}


byte LSM303_read(byte address)
{
  byte temp;
  
  if (address >= 0x20)
    Wire.beginTransmission(0x18);
  else
    Wire.beginTransmission(0x1E);
    
  Wire.write(address);
  
  if (address >= 0x20)
    Wire.requestFrom(0x18, 1);
  else
    Wire.requestFrom(0x1E, 1);
  while(!Wire.available())
    ;
  temp = Wire.read();
  Wire.endTransmission();
  
  return temp;
}



void LSM303_write(byte data, byte address)
{
  if (address >= 0x20)
    Wire.beginTransmission(0x18);
  else
    Wire.beginTransmission(0x1E);
    
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}


/* End code */
