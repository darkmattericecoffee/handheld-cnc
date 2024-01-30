// Libraries to include
#include <PMW3360.h>
#include <SPI.h>
#include <SD.h>

// Calibration process:
//      1. get out x and y calibration rails
//      2. secure the rails to a clean surface
//      3. connect sensors to your microcontroller and begin script
//      4. select a sensor and place it in the x rail
//      5. type into the serial port the number of that sensor that you want to calibrate
//          ex: '2' for sensor number 2
//      6. start with the sensor at one end of the rail, with the arrow pointing forward
//      7. move it slowly to the other end, trying to keep a constant speed and downward pressure
//      8. stop briefly, then move it back to the origin
//      9. stop briefly, then repeat steps 7-8 4 more times
//      10. move sensor to the y rail and repeat steps 6-9
//      11. repeat steps 4-10 for the rest of the sensors
//      12. type 'w' then 'a' into the serial port to write all calibration values to EEPROM
//          a. type 'w' then the number of a specific sensor to just write that sensor's value

// Function definitions
// Setup functions
void sensorSetup();
// Math functions
int convTwosComp(int b);
float mapF(long x, float in_min, float in_max, float out_min, float out_max);
// Sensing functions
void sensorPlotting();
void doSensing();

// Pin definitions -------------------------------------------------------------------------------
// Sensor pins
#define SS0   39   // Chip select pin. Connect this to SS on the module.
#define SS1   10
#define SS2   40
#define SS3   32   // ...if working

// Constants ------------------------------------------------------------------------
// Modes
int plotting = 0;             // plot values  (1 = yes; 0 = no)
int debugMode = 1;            // print values (1 = yes; 0 = no)

// Timing properties
long unsigned debounceDelay = 50;      // the debounce time; increase if the output flickers
long unsigned dtDebug = 500;            // [ms]
long unsigned dtStop = 1000;            // time to indicate a stop [ms]

// Sensor properties
const int ns = 4;                   // number of sensors
const int CPI = 2500;               // This value changes calibration coefficients
long unsigned dt = 500;       // microseconds (freq = 1,000,000/timestepPoll [Hz])

// Calibration properties
float l = 100.0f;             // length of track [mm]
float zeroThresh = 500.0f;    // threshold to detmerine zero [unitless]
const int numRuns = 5;            // number of calibration runs

// Variables ------------------------------------------------------------------------
// Timing variables
int firstPoint = 1;
long unsigned timeLastPoll = 0;
long unsigned timeLastDebug = 0;
long unsigned timeLastDebounce = 0;   // (ms)
long unsigned timeSinceGo = 0;

// Measured quantities
float measVelX[4] = {0.0f,0.0f,0.0f,0.0f};     // BFF x velocity of each sensor (mm/us)
float measVelY[4] = {0.0f,0.0f,0.0f,0.0f};     // BFF y velocity of each sensor (mm/us)
float estPosX[4] = {0.0f,0.0f,0.0f,0.0f};
float estPosY[4] = {0.0f,0.0f,0.0f,0.0f};

// Calibration variables
int sensorSelect = -1;
float Cx[4];            // calibration coefficient to be calculated
float Cy[4];
int runCount = 0;
bool atOrigin = 1;
bool xMode = 1;
float calValHolder[numRuns];
bool runInitiated = 0;

// Object Initialization ------------------------------------------------------------
// Sensor object creation
PMW3360 sensor0;
PMW3360 sensor1;
PMW3360 sensor2;
PMW3360 sensor3;

// -------------------------------------------------------------------------------------------------
// Setup and Main Loop -----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  delay(100);

  // Setup systems
  sensorSetup();

  Serial.println("Calibration ready!");
  
  delay(500);
}

void loop() {
  // Serial Interface -----------------------------------------------------------------------------
  if (Serial.available()) {
    char ch = Serial.read();
    switch(ch) {
      case '0':
        sensorSelect = 0;
        break;
      case '1':
        sensorSelect = 1;
        break;

      // To-Do: add more cases
    }
  }

  if (sensorSelect != -1 && runCount < numRuns) {
    if(micros() - timeLastPoll >= dt) {
      doSensing();

      if (xMode) {
        // Check whether the chip has left origin
        if (estPosX[sensorSelect] > zeroThresh) { 
          atOrigin = 0;

          // See if stopped
          if (millis() - timeSinceGo >= dtStop) {
            calValHolder[runCount] = estPosX[sensorSelect];
            runInitiated = 0;
          }
        } else {
          atOrigin = 1;

          if (millis() - timeSinceGo >= dtStop && !runInitiated) {
            estPosX[sensorSelect] = 0;      // reset sensor position
            runInitiated = 1;
            runCount += 1;

            Serial.printf("Ready for run %i\n", runCount + 1);

          }
        }

        if (runCount == numRuns){
          // Calculate calibration value
          // TO-DO ^^

          Serial.printf("X calibration for sensor %i done. Value = ", sensorSelect);
          Serial.println("Move sensor to Y rail and press any key when ready");

          while (!Serial.available());
          char ch = Serial.read();

          // Switch to Y calibration
          xMode = 0;
          atOrigin = 1;
          runCount = 0;
          runInitiated = 1;
        }
      } else {
        // Y mode now
        // Check whether the chip has left origin
        if (estPosY[sensorSelect] > zeroThresh) { 
          atOrigin = 0;

          // See if stopped
          if (millis() - timeSinceGo >= dtStop) {
            calValHolder[runCount] = estPosY[sensorSelect];
            runInitiated = 0;
          }
        } else {
          atOrigin = 1;

          if (millis() - timeSinceGo >= dtStop && !runInitiated) {
            estPosY[sensorSelect] = 0;      // reset sensor position
            runInitiated = 1;
            runCount += 1;

            Serial.printf("Ready for run %i\n", runCount + 1);

          }
        }

        if (runCount == numRuns){
          // Calculate calibration value
          // TO-DO ^^

          Serial.printf("X calibration for sensor %i done. Value = ", sensorSelect);
          Serial.println("Move sensor to Y rail and press any key when ready");

          while (!Serial.available());
          char ch = Serial.read();

          // Reset calibration
          xMode = 1;
          atOrigin = 1;
          runCount = 0;
          runInitiated = 1;
        }
      }

    }


  }
}

// ------------------------------------------------------------------------------------------------
// Setup subfunctions -----------------------------------------------------------------------------
void sensorSetup() {
  // Sensor initialization
  if(sensor0.begin(SS0, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor0 initialization successed, with CPI = ");
    Serial.println(sensor0.getCPI());
  }else{
    Serial.println("Sensor0 initialization failed");
  }
  if(sensor1.begin(SS1, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor1 initialization successed, with CPI = ");
    Serial.println(sensor1.getCPI());
  }else{
    Serial.println("Sensor1 initialization failed");
  }
  if(sensor2.begin(SS2, CPI)) {  // 10 is the pin connected to SS of the module.
    Serial.print("Sensor2 initialization successed, with CPI = ");
    Serial.println(sensor2.getCPI());
  }else{
    Serial.println("Sensor2 initialization failed");
  }
}

// Loop subfunctions -----------------------------------------------------------------------------
// Math functions
int convTwosComp(int b){
  // Convert from 2's complement
  // This is needed to convert the binary values from the sensor library to decimal

  if(b & 0x80){                     // 0x80 (hex) = 10000000 (bin) = 128 (dec)
    b = -1 * ((b ^ 0xff) + 1);      // 0xff (hex) = 11111111 (bin) = 255 (dec)
  }
  return b;
}

// Sensing functions
void doSensing() {
  timeLastPoll = micros();

  // Sensing ---------------------------------------------------------------------
  // Collect sensor data (raw)
  PMW3360_DATA data0 = sensor0.readBurst_simple();
  PMW3360_DATA data1 = sensor1.readBurst_simple();
  PMW3360_DATA data2 = sensor2.readBurst_simple();
  PMW3360_DATA data3 = sensor3.readBurst_simple();

  int yup = 1;        // filling in for good motion detection boolean
  if(yup) {
  //if(data1.isOnSurface && data1.isMotion) {   // If movement...
    // **Only checking sensor 1 rn (TODO: check all of them and account for misreads)**

    // Sensor velocity sensing
    measVelX[0] = -convTwosComp(data0.dx) / dt;     // is '-' necessary?
    measVelX[1] = -convTwosComp(data1.dx) / dt;
    measVelX[2] = -convTwosComp(data2.dx) / dt;
    measVelX[3] = -convTwosComp(data3.dx) / dt;
    measVelY[0] = convTwosComp(data0.dy) / dt;
    measVelY[1] = convTwosComp(data1.dy) / dt;
    measVelY[2] = convTwosComp(data2.dy) / dt;
    measVelY[3] = convTwosComp(data3.dy) / dt;

    // Record time stamp of movement
    // To-Do: make sure a reading is actually sent when the sensor is stopped
    if (xMode){
      if (measVelX[sensorSelect] > 0) {
        timeSinceGo = millis();
      }
    } else {
      if (measVelY[sensorSelect] > 0) {
        timeSinceGo = millis();
      }
    }

    // Integrate linear velocities to get position
    for (int i = 0; i < 4; i++) {
      estPosX[i] = estPosX[i] + measVelX[i]*dt;
      estPosY[i] = estPosY[i] + measVelY[i]*dt;
    }

    // Sensor plotting
    if (plotting) {
      sensorPlotting();
    }
  }
}

void sensorPlotting() {
  // Plot sensor data

  //Serial.printf("dx:%f,dy:%f",measVelX,measVelY);
  //Serial.printf("dx:%i,dy:%i",data.dx,data.dy);
  Serial.printf("x:%f,y:%f",estPosX,estPosY);
//      Serial.printf("w1:%f,w2:%f,w3:%f,w4:%f,w5:%f,w6:%f,w7:%f,w8:%f",estAngVel[0],estAngVel[1],
//        estAngVel[2],estAngVel[3],estAngVel[4],estAngVel[5],estAngVel[6],estAngVel[7]);
  //Serial.printf("x:%f,y:%f",xmm[1],ymm[1]);
  Serial.println();
}

void debugging() {
  // Print debug data
  // Put all Serial print lines here to view
  
  // if you need to debug...

  Serial.println();
}