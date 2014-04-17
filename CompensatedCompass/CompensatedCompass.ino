
#include <Wire.h>
#include <math.h>

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

/* Global variables */
int mag[3];  // raw magnetometer values stored here

void setup()
{
  Serial.begin(9600);  // Serial is used for debugging
  Wire.begin();  // Start up I2C, required for LSM303 communication
  initLSM303(ACCELE_SCALE);  // Initialize the LSM303, using a SCALE full-scale range
}

void loop()
{
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
  Serial.println("The clockwise angle between the magnetic north and x-axis: ");
  Serial.print(getHeading(mag), 3); // this only works if the sensor is level
  Serial.println(" degrees");
  delay(200);  // delay for serial readability
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

/*
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

*/

/*
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
*/
/*
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

*/


/*
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
*/
