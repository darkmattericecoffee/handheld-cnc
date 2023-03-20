#include <PMW3360.h>
/*
Sensor configuration:
2 --- 4           ^ y (unsure whether or not x and y are flipped)
|  x  |       x <-|
1 --- 3
*/

#define SS1   10   // Slave Select pin. Connect this to SS on the module.
#define SS2   39
#define SS3   40
#define SS4   41


// Sensor properties
int CPI = 2500;               // This value changes calibration coefficients
float Cx[4] = {0.01003310926f,0.00996611521f,0.01008674602f,0.00997506234f};
float Cy[4] = {0.01026588646f,0.01019056354f,0.01016570093f,0.01011531459f};
float l = 100;                // length of square sensor configuration

// Variables ------------------------------------------------------------------------
int plotting = 1;

int firstPoint = 1;
long timeLastPoll = 0;
long dt = 100;       // microseconds (freq = 1,000,000/timestepPoll [Hz])

// Measured quantities
int dX[4] = {0,0,0,0};
int dY[4] = {0,0,0,0};
float dXmm[4] = {0f,0f,0f,0f};
float dYmm[4] = {0f,0f,0f,0f};
float vXmm[4] = {0f,0f,0f,0f};        // velocity (mm/s)(dXmm/dt)
float vYmm[4] = {0f,0f,0f,0f};

// Estimated quantities
float xmm[4] = {0f,0f,0f,0f};
float ymm[4] = {0f,0f,0f,0f};
float estPosX[6] = {0f,0f,0f,0f,0f,0f};      // Position estimation from each sensor combination
float estPosY[6] = {0f,0f,0f,0f,0f,0f};      // Position estimation from each sensor combination
float estAng[6] = {0f,0f,0f,0f,0f,0f};      // Angle estimation from each sensor combination

// Control Constants ----------------------------------------------------------------

// Control Variables ----------------------------------------------------------------

// Object Initialization ------------------------------------------------------------
PMW3360 sensor1;
PMW3360 sensor2;
PMW3360 sensor3;
PMW3360 sensor4;

void setup() {
  Serial.begin(9600);  
  while(!Serial);
  
  // Sensor initialization
  if(sensor.begin(SS1, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor1 initialization successed, with CPI = ");
    Serial.println(sensor.getCPI());
  }else{
    Serial.println("Sensor1 initialization failed");
  }
  if(sensor.begin(SS2, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor2 initialization successed, with CPI = ");
    Serial.println(sensor.getCPI());
  }else{
    Serial.println("Sensor2 initialization failed");
  }
  if(sensor.begin(SS3, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor3 initialization successed, with CPI = ");
    Serial.println(sensor.getCPI());
  }else{
    Serial.println("Sensor3 initialization failed");
  }
  if(sensor.begin(SS4, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor4 initialization successed, with CPI = ");
    Serial.println(sensor.getCPI());
  }else{
    Serial.println("Sensor4 initialization failed");
  }
   
}

void loop() {
  if(micros() - timeLastPoll >= dt) {
    timeLastPoll = micros();

    // Sensing ---------------------------------------------------------------------
    // Collect sensor data
    PMW3360_DATA data1 = sensor1.readBurst_simple();
    PMW3360_DATA data2 = sensor2.readBurst_simple();
    PMW3360_DATA data3 = sensor3.readBurst_simple();
    PMW3360_DATA data4 = sensor4.readBurst_simple();
    
    if(data1.isOnSurface && data1.isMotion) {   // If movement...
      // Only checking sensor 1 rn (TODO: check all of them and account for misreads)

      // Sensor velocity sensing
      dXmm[0] = convTwosComp(data1.dx)*Cx[0];
      dXmm[1] = convTwosComp(data2.dx)*Cx[1];
      dXmm[2] = convTwosComp(data3.dx)*Cx[2];
      dXmm[3] = convTwosComp(data4.dx)*Cx[3];
      dYmm[0] = convTwosComp(data1.dy)*Cx[0];
      dYmm[1] = convTwosComp(data2.dy)*Cx[1];
      dYmm[2] = convTwosComp(data3.dy)*Cx[2];
      dYmm[3] = convTwosComp(data4.dy)*Cx[3];

//      dXmm = dX*Cx[3];
//      dYmm = dY*Cy[3];

      // Sensor position estimation
      if(firstPoint) {
        // make sure first point (x,y) is (0,0) (there could be a better way to do this)
        firstPoint = 0;
      }else{
        for (int i = 0; i<4; i++) {
          xmm[i] = xmm[i] + dXmm[i];
          ymm[i] = ymm[i] + dYmm[i];
        }
      }

      // Body position estimation
      estPos[0] = xmm[0] + 
      
      if(plotting) {
        Serial.printf("dx:%f,dy:%f,",dXmm,dYmm);
        //Serial.printf("dx:%i,dy:%i,",data.dx,data.dy);
        Serial.printf("x:%f,y:%f",xmm,ymm);
        Serial.println();
      } else {
        Serial.print(data.dx);
        Serial.print("\t");
        Serial.print(data.dy);
        Serial.println();
      }
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
