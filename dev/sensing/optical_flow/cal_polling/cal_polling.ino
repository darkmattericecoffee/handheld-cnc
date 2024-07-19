// Libraries to include
#include <PMW3360.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

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
int16_t convTwosComp(int16_t value);
float mapF(long x, float in_min, float in_max, float out_min, float out_max);
// Sensing functions
void sensorPlotting();
void debugging();
void doSensing();

// Pin definitions -------------------------------------------------------------------------------
// Sensor pins
#define SS0   39   // Chip select pin. Connect this to SS on the module.
#define SS1   10
#define SS2   40
#define SS3   32   // ...if working
int sensorPins[4] = {SS0, SS1, SS2, SS3};

// Constants ------------------------------------------------------------------------
// Modes
int plotting = 0;             // plot values  (1 = yes; 0 = no)
int debugMode = 1;            // print values (1 = yes; 0 = no)

// Timing properties
long unsigned debounceDelay = 50;      // the debounce time; increase if the output flickers
long unsigned dtDebug = 500;            // [ms]
long unsigned dtStop = 2000;            // time to indicate a stop [ms]

// Sensor properties
const int ns = 4;                   // number of sensors
const int CPI = 2500;               // This value changes calibration coefficients
long unsigned dt = 500;       // microseconds (freq = 1,000,000/timestepPoll [Hz])

// Calibration properties
float vel_tolerance = 0;
float l = 100.0f;             // length of track [mm]
float zeroThresh = 2000.0f;    // threshold to detmerine zero [unitless](100units ~= 1mm)
const int numRuns = 5;            // number of calibration runs

// Variables ------------------------------------------------------------------------
// Timing variables
int firstPoint = 1;
long unsigned timeLastPoll = 0;
long unsigned timeLastDebug = 0;
long unsigned timeLastDebounce = 0;   // (ms)
long unsigned timeSinceGo = 0;

// Measured quantities
float measVel[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                      {0.0f,0.0f,0.0f,0.0f}};     // BFF (x,y) velocity of each sensor (mm/us)
bool mot[4] = {0, 0, 0, 0};
float estPos[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                      {0.0f,0.0f,0.0f,0.0f}}; 

// Calibration variables
int sensorSelect = -1;
float cVal[2][4];            // calibration coefficient to be calculated
int runCount = 0;
bool atOrigin = 1;
int axis = 0;                 // axis to calibrate (0 = x; 1 = y)
float calValHolder[numRuns];
bool runInitiated = 0;

// Object Initialization ------------------------------------------------------------
// Sensor object creation
PMW3360 sensors[4];

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
    if (ch >= '0' && ch <= '3') {
      int index = ch - '0';         // convert char to int by subtracting ASCII value of '0'
      sensorSelect = index;
      runInitiated = 1;
      Serial.printf("Starting X calibration with sensor %i\n", sensorSelect);
      Serial.println("Move sensor to the other end of the calibration rail and keep it there for 3 seconds");
    } else if (ch == 'w') {         // TO-DO: change to switch case if more inputs are needed
      //writeToEEPROM();
    }
  }

  if(micros() - timeLastPoll >= dt) {
    doSensing();
  }

  debugging();
}

// ------------------------------------------------------------------------------------------------
// Setup subfunctions -----------------------------------------------------------------------------
void sensorSetup() {
  // Sensor initialization
  for (int i = 0; i < 4; i++) {
    if (sensors[i].begin(sensorPins[i], CPI)) {
      Serial.print("Sensor");
      Serial.print(i);
      Serial.print(" initialization succeeded, with CPI = ");
      Serial.println(sensors[i].getCPI());
    } else {
      Serial.print("Sensor");
      Serial.print(i);
      Serial.println(" initialization failed");
    }
  }
}

// Loop subfunctions -----------------------------------------------------------------------------
// Math functions
int16_t convTwosComp(int16_t value){
  // Convert from 2's complement (16 bit now)
  // This is needed to convert the binary values from the sensor library to decimal

  if (value & 0x8000) {                     // Check if the sign bit is set (negative value)
    return -((~value & 0xFFFF) + 1);      // Invert bits, add one, make negative
  } else {
    return value;
  }
}

// Sensing functions
void doSensing() {
  timeLastPoll = micros();

  // Sensing ---------------------------------------------------------------------
  // Collect sensor data (raw)
  PMW3360_DATA data[4];
  for (int i = 0; i < 4; i++) {
    data[i] = sensors[i].readBurst();
  }

  // Record time stamp of movement
  if (data[sensorSelect].isMotion == 1) {
    timeSinceGo = millis();
  }

  for (int i = 0; i < 4; i++) {
    // Sensor velocity sensing
    measVel[0][i] = -convTwosComp(data[i].dx);     // '-' convention is used to flip sensor's z axis
    measVel[1][i] = convTwosComp(data[i].dy);

    // Integrate linear velocities to get position
    estPos[0][i] = estPos[0][i] + measVel[0][i];
    estPos[1][i] = estPos[1][i] + measVel[1][i];

    mot[i] = data[i].isMotion;
  }

  // Sensor plotting
  if (plotting) {
    sensorPlotting();
  }
}

void sensorPlotting() {
  // Plot sensor data

  Serial.printf("x_raw:%f,y_raw:%f",measVel[0][sensorSelect],measVel[1][sensorSelect]);
  // Serial.printf("dx:%i,dy:%i",data.dx,data.dy);
  // Serial.printf("x:%f,y:%f",estPos[0][sensorSelect],estPos[1][sensorSelect]);
  //    Serial.printf("w1:%f,w2:%f,w3:%f,w4:%f,w5:%f,w6:%f,w7:%f,w8:%f",estAngVel[0],estAngVel[1],
  //      estAngVel[2],estAngVel[3],estAngVel[4],estAngVel[5],estAngVel[6],estAngVel[7]);
  // Serial.printf("x:%f,y:%f",xmm[1],ymm[1]);
  Serial.println();
}

void debugging() {
  // Print debug data
  // Put all Serial print lines here to view
  if(millis() - timeLastDebug >= dtDebug && debugMode) {
    timeLastDebug = millis();
    
    // if you need to debug...
    Serial.printf("dx:%f,dy:%f",measVel[0][sensorSelect],measVel[1][sensorSelect]);
    Serial.print(",");
    Serial.printf("x:%f,y:%f",estPos[0][sensorSelect],estPos[1][sensorSelect]);
    Serial.print(",");
    Serial.printf("mot:%i", mot[sensorSelect]);
    Serial.print(",");
    Serial.println();
  }
}