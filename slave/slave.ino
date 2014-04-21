
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <Wire.h>

// Ladyada's logger modified by Bill Greiman to use the SdFat library
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS Shield
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

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


void setup() {
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println(freeRam());
  Wire.begin(2);                // join i2c bus with address #4
  Wire.onRequest(requestEvent);
  
  //Serial.println("\r\nUltimate GPSlogger Shield");
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect, 11, 12, 13)) {
  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("f");
    exit(2);
  }

  //Serial.print("Writing to "); Serial.println(filename);
  
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
}

void loop() {

    //Serial.print("R");
  Serial.println(freeRam());
  
  logfile = SD.open("0", FILE_WRITE);
  if( ! logfile ) {
    Serial.println("C"); //Serial.println(filename);
    exit(3);
  }
  
  
  
  while(gps_receive()==0);
  
  
  
  //Serial.println((int)GPS.longitude);
  requestEvent();
  
  
  logfile.close();
  delay(1000);
  
}


byte gps_receive(){
  char c = GPS.read();
  if (GPSECHO)
     if (c)   ;//Serial.print(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return 0;  // we can fail to parse a sentence in which case we should just wait for another
    
    // Sentence parsed! 
    //Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
        //Serial.print("No Fix");
        return 0;
    }

    // Rad. lets log it!
    //Serial.println("Log");
    
    char *stringptr = GPS.lastNMEA();
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
      exit(4);
    if (strstr(stringptr, "RMC"))   logfile.flush();
    return 1;
  } 
  return 0;
}





void requestEvent(){
  
  char data[12] = {};
  char temp[5] = {};
  
  itoa((int)GPS.latitude, temp,10);
  //if(GPS.lat == 'S') data[0] = '0';
  strcat(data, temp);
  
  itoa((int)GPS.longitude, temp,10);
  //if(GPS.lon == 'W') data[0] = '0';
  strcat(data, temp);
  
  strcat(data, "\n");
  
  Wire.write(data);
  
  
}


/* End code */
