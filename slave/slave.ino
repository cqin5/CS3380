#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <math.h>
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
 // if (!SD.begin(chipSelect, 11, 12, 13)) {
  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("1"); // Car init. failed
    exit(1);
  }


  char filename[1];
  strcpy(filename, "0");
  for (uint8_t i = 0; i < 10; i++) {
    filename[0] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  //If ram drops below 300 bytes, it wouldnt be able to continue. 
  //Serial.print("R:");
  //Serial.println(freeRam());

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("2"); // Couldn't write to file
    exit(1);
  }

  // connect to the GPS at the desired rate
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 or 5 Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);

  //Serial.println("Ready!");



}


void loop() {
  get_gps_data();
  Serial.println();
  Serial.println(get_distance());

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
        exit(1);
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
// get_satellites:      6

double get_gps_data(int index){
  switch(index) {
  case 10:      
    return GPS.latitude/100;
  case 11:      
    if(GPS.lat == 'N') return  1.0;
    else               return -1.0;

  case 20:      
    return GPS.longitude/100;
  case 21:      
    if(GPS.lon == 'E') return  1.0;
    else               return -1.0;

  case 3:       
    return GPS.latitude;
  case 4:       
    return GPS.speed;
  case 5:       
    return GPS.magvariation;
  case 6:       
    return GPS.satellites;
  }
  return 999.99;
}


// haversine formula
double get_distance(){
  // Richmond St. and Twelve Mile Rd, ~ 12 km from Western
  float latitude_destination  =  43.11;
  float longitude_destination = -81.33;

  float latitude_current  = GPS.latitude/100;//  * get_gps_data(11);
  float longitude_current = GPS.longitude/100 * -1;//get_gps_data(21); 

  if(latitude_current == 0.00 && longitude_current == 0.00){
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

    //dist_calc *= 6371000.0; //Converting to meters
    //Serial.print("D:");
    //Serial.println(dist_calc);
    return dist_calc * 6371000.0; //Converting to meters
  }
}



/* End code */
