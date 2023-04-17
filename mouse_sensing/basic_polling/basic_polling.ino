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

// Sensor properties
int CPI = 2500;               // This value changes calibration coefficients
float Cx[4] = {0.01003310926f,0.00996611521f,0.01008674602f,0.00997506234f};
float Cy[4] = {0.01026588646f,0.01019056354f,0.01016570093f,0.01011531459f};

int mmMode = 1;
int x = 0;
int y = 0;
float xmm = 0;
float ymm = 0;
int dX = 0;
int dY = 0;
float dXmm = 0;
float dYmm = 0;
int plotting = 1;

int firstPoint = 1;
long timeLastPoll = 0;
long timestepPoll = 100;       // microseconds (freq = 1,000,000/timestepPoll [Hz])

PMW3360 sensor;

void setup() {
  Serial.begin(9600);  
  while(!Serial);
  
  //sensor.begin(10, 1600); // to set CPI (Count per Inch), pass it as the second parameter
  if(sensor.begin(SS, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor initialization successed, with CPI = ");
    Serial.println(sensor.getCPI());
  }else{
    Serial.println("Sensor initialization failed");
  }

  //sensor.setCPI(12000);
  //Serial.println(sensor.getCPI());
  
  
  //sensor.setCPI(1600);    // or, you can set CPI later by calling setCPI();
}

void loop() {
  if(micros() - timeLastPoll >= timestepPoll) {
    timeLastPoll = micros();
    // Collect sensor data
    PMW3360_DATA data = sensor.readBurst_simple();
    
    if(data.isOnSurface && data.isMotion) {
      // If there is movement...
      dX = convTwosComp(data.dx);
      dY = convTwosComp(data.dy);
      if(mmMode) {
        dXmm = dX*Cx[3];
        dYmm = dY*Cy[3];
      }

      if(firstPoint) {
        firstPoint = 0;
      }else{
        if(mmMode) {
          xmm = xmm + dXmm;
          ymm = ymm + dYmm;
        }else{
          x = x + dX;
          y = y + dY;
        }
      }
      
      if(plotting) {
        if(mmMode) {
          Serial.printf("dx:%f,dy:%f,",dXmm,dYmm);
          //Serial.printf("dx:%i,dy:%i,",data.dx,data.dy);
          Serial.printf("x:%f,y:%f",xmm,ymm);
          Serial.println();
        }else{
          Serial.printf("dx:%i,dy:%i,",dX,dY);
          //Serial.printf("dx:%i,dy:%i,",data.dx,data.dy);
          Serial.printf("x:%i,y:%i",x,y);
          Serial.println();
        }
      } else {
        Serial.print(data.dx);
        Serial.print("\t");
        Serial.print(data.dy);
        Serial.println();
      }
  //    Serial.print("dt=");
  //    Serial.println(micros() - timeStart);
    }
  }
  
  //delay(10);
}


int convTwosComp(int b){
  //Convert from 2's complement
  //if(sizeof(b) > 
  if(b & 0x80){                     // 0x80 (hex) = 10000000 (bin) = 128 (dec)
    b = -1 * ((b ^ 0xff) + 1);      // 0xff (hex) = 11111111 (bin) = 255 (dec)
  }
  return b;
}
