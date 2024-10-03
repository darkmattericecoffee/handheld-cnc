// Libraries to include
#include <PMW3360_router.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// Calibration process:

// Function definitions
// Setup functions
void sensorSetup();
void wait_for_movement();
// Math functions
int16_t convTwosComp(int16_t value);
float mapF(long x, float in_min, float in_max, float out_min, float out_max);
float meanArray(float* arr, int size);
// Sensing functions
void sensorPlotting();
void debugging();
void doSensing();
// Other loop functions
void writeToEEPROM();
void eepromWrite(int address, float value);
float eepromRead(int address);

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

// Variables ------------------------------------------------------------------------
// Timing variables
int firstPoint = 1;
long unsigned timeLastPoll = 0;
long unsigned timeLastDebug = 0;

// Measured quantities
float measVel[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                      {0.0f,0.0f,0.0f,0.0f}};     // BFF (x,y) velocity of each sensor (mm/us)
float estPos[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                      {0.0f,0.0f,0.0f,0.0f}}; 

// Calibration variables
int sensorSelect = -1;
float cVal[2][4];            // calibration coefficient to be calculated
int axis = 0;                 // axis to calibrate (0 = x; 1 = y)

// Object Initialization ------------------------------------------------------------
// Sensor object creation
PMW3360 sensors[4];

// -------------------------------------------------------------------------------------------------
// Setup and Main Loop -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  while(!Serial);         // we actually want to make sure serial is opened for calibration
  delay(100);

  Serial.println("Serial set up!");

  // Setup systems
  sensorSetup();

  Serial.println("Calibration ready!");
  
  delay(500);
}

void loop() {
  // Serial Interface -----------------------------------------------------------------------------
  // Serial.println("in loop");
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch >= '0' && ch <= '3') {
      int index = ch - '0';         // convert char to int by subtracting ASCII value of '0'
      sensorSelect = index;
      delay(500);
      estPos[0][sensorSelect] = 0.0;
      estPos[1][sensorSelect] = 0.0;
      Serial.printf("Starting X calibration with sensor %i\n", sensorSelect);
      Serial.println("Move sensor to the other end of the calibration rail and keep it there for 3 seconds");
    }
  }

  if (sensorSelect != -1) {
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
  if(micros() - timeLastPoll >= dt) {
    timeLastPoll = micros();

    // Sensing ---------------------------------------------------------------------
    // Collect sensor data (raw)
 
    PMW3360_DATA data[4];
    for (int i = 0; i < 4; i++) {
      data[i] = sensors[i].readBurst();
    }

    for (int i = 0; i < 4; i++) {
      // Sensor velocity sensing
      // measVel[0][i] = -convTwosComp(data[i].dx);     // '-' convention is used to flip sensor's z axis
      measVel[0][i] = convTwosComp(data[i].dx);
      measVel[1][i] = convTwosComp(data[i].dy);

      // Integrate linear velocities to get position
      estPos[0][i] = estPos[0][i] + measVel[0][i];
      estPos[1][i] = estPos[1][i] + measVel[1][i];
    }

    // Sensor plotting
    if (plotting) {
      sensorPlotting();
    }
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
    Serial.printf("x:%f,y:%f",estPos[0][sensorSelect],estPos[1][sensorSelect]);
    Serial.println();
  }
}