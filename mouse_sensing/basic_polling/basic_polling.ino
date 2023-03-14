#include <PMW3360.h>

/* 

# PIN CONNECTION
  * MI = MISO
  * MO = MOSI
  * SS = Slave Select / Chip Select
  * SC = SPI Clock
  * MT = Motion (active low interrupt line)
  * RS = Reset
  * GD = Ground
  * VI = Voltage in up to +5.5V 

Module   Arduino
  RS --- (NONE)
  GD --- GND
  MT --- (NONE)
  SS --- Pin_10   (use this pin to initialize a PMW3360 instance)
  SC --- SCK 
  MO --- MOSI
  MI --- MISO
  VI --- 5V

# PMW3360_DATA struct format and description
  - PMW3360_DATA.isMotion      : bool, True if a motion is detected. 
  - PMW3360_DATA.isOnSurface   : bool, True when a chip is on a surface 
  - PMW3360_DATA.dx, data.dy   : integer, displacement on x/y directions.
  - PMW3360_DATA.SQUAL         : byte, Surface Quality register, max 0x80
                               * Number of features on the surface = SQUAL * 8
  - PMW3360_DATA.rawDataSum    : byte, It reports the upper byte of an 18‐bit counter 
                               which sums all 1296 raw data in the current frame;
                               * Avg value = Raw_Data_Sum * 1024 / 1296
  - PMW3360_DATA.maxRawData    : byte, Max/Min raw data value in current frame, max=127
    PMW3360_DATA.minRawData
  - PMW3360_DATA.shutter       : unsigned int, shutter is adjusted to keep the average
                               raw data values within normal operating ranges.
 */

#define SS  10   // Slave Select pin. Connect this to SS on the module.

int x = 0;
int y = 0;
int dX = 0;
int dY = 0;
int plotting = 1;

PMW3360 sensor;

void setup() {
  Serial.begin(9600);  
  while(!Serial);
  
  //sensor.begin(10, 1600); // to set CPI (Count per Inch), pass it as the second parameter
  if(sensor.begin(SS))  // 10 is the pin connected to SS of the module.
    Serial.println("Sensor initialization successed");
  else
    Serial.println("Sensor initialization failed");
  
  //sensor.setCPI(1600);    // or, you can set CPI later by calling setCPI();
}

void loop() {
  PMW3360_DATA data = sensor.readBurst_simple();
  
  if(data.isOnSurface && data.isMotion)
  {
//    if(data.dx & 0x80) {
//      data.dx = -1 * ((data.dx ^ 0xff) + 1);
//    }
//    if(data.dy & 0x80) {
//      data.dy = -1 * ((data.dy ^ 0xff) + 1);
//    }
    dX = convTwosComp(data.dx);
    dY = convTwosComp(data.dy);

    x = x + dX;
    y = y + dY;
    
    if(plotting) {
      //Serial.printf("dx:%i,dy:%i,",dX,dY);
      Serial.printf("dx:%i,dy:%i,",data.dx,data.dy);
      Serial.printf("x:%i,y:%i",x,y);
      Serial.println();
    } else {
      Serial.print(data.dx);
      Serial.print("\t");
      Serial.print(data.dy);
      Serial.println();

//      Serial.printf("xy = (%i,", xy[0]);
//      Serial.printf(" %i)", xy[1]);
//      Serial.println();
//      Serial.println();
    }
  }
  
  delay(10);
}


int convTwosComp(int b){
  //Convert from 2's complement
  //if(sizeof(b) > 
  if(b & 0x80){                     // 0x80 (hex) = 10000000 (bin) = 128 (dec)
    b = -1 * ((b ^ 0xff) + 1);      // 0xff (hex) = 11111111 (bin) = 255 (dec)
  }
  return b;
}
