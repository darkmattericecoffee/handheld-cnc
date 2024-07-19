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

// Calibration properties
float vel_tolerance = 1;
float l = 100.0f;             // length of track [mm]
float zeroThresh = 2000.0f;    // threshold to detmerine zero [unitless](100units ~= 1mm)
const int numRuns = 5;            // number of calibration runs

// Variables ------------------------------------------------------------------------
// Timing variables
int firstPoint = 1;
long unsigned timeLastPoll = 0;
long unsigned timeLastDebug = 0;
long unsigned timeLastDebounce = 0;   // (ms)
long unsigned timeLastGo = 0;

// Measured quantities
float measVel[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                      {0.0f,0.0f,0.0f,0.0f}};     // BFF (x,y) velocity of each sensor (mm/us)
float estPos[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                      {0.0f,0.0f,0.0f,0.0f}}; 

// Calibration variables
int sensorSelect = -1;
float cVal[2][4];            // calibration coefficient to be calculated
int currentRun = 0;
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

  while(!Serial);         // we actually want to make sure serial is opened for calibration
  // delay(100);

  // Setup systems
  sensorSetup();

  // Serial.println("Calibration ready!");
  
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
      runInitiated = 1;
      estPos[0][sensorSelect] = 0.0;
      estPos[1][sensorSelect] = 0.0;
      Serial.printf("Starting X calibration with sensor %i\n", sensorSelect);
      Serial.println("Move sensor to the other end of the calibration rail and keep it there for 3 seconds");
      wait_for_movement();
    } else if (ch == 'w') {         // TO-DO: change to switch case if more inputs are needed
      writeToEEPROM();
    }
  }

  if (sensorSelect != -1 && currentRun < numRuns) {
    // Serial.println("entering if statmennt");
    doSensing();

    // Check whether the chip has left origin
    if (abs(estPos[axis][sensorSelect]) > zeroThresh) { 
      // atOrigin = 0;
      // See if stopped
      if (millis() - timeLastGo >= dtStop && runInitiated) {
        calValHolder[currentRun] = estPos[axis][sensorSelect];
        runInitiated = 0;
        Serial.println("Ok, now move it back to origin.");
      }
    } else {
      // atOrigin = 1;
      // See if stopped 
      debugging();
      if (millis() - timeLastGo >= dtStop && !runInitiated) {
        runInitiated = 1;
        currentRun += 1;          // base 0
        estPos[axis][sensorSelect] = 0;      // reset sensor position (we want to reset before wait_for_movement)
        Serial.printf("Ready for run %i/%i.\n Move sensor to end of rail again.\n", currentRun + 1, numRuns);     // print base 1
        // This waits until we start moving to move through loop() again
        wait_for_movement();
      }
    }
  } else if (currentRun == numRuns) {
    // Once runs have completed...
    if (!axis){
      // Calculate calibration value
      cVal[axis][sensorSelect] = l / meanArray(calValHolder, numRuns);

      Serial.printf("X calibration for sensor %i done. Value = %.2f\n", sensorSelect, cVal[axis][sensorSelect]);
      Serial.println("Move sensor to Y rail and press any key when ready");

      while (!Serial.available());
      char ch = Serial.read();          // To-Do: maybe we can just do Serial.read() instead to skirt the warning
      Serial.println("Ok awesome! Now move sensor to end of rail and stop.");

      // Switch to Y calibration
      axis = 1;
      currentRun = 0;
      runInitiated = 1;
    } else if (axis) {
      // Calculate calibration value
      cVal[axis][sensorSelect] = l / meanArray(calValHolder, numRuns);

      Serial.printf("Y calibration for sensor %i done. Value = %.2f\n", sensorSelect, cVal[axis][sensorSelect]);

      // Reset calibration
      axis = 0;
      currentRun = 0;
      runInitiated = 0;
      sensorSelect = -1;        // reset sensor select to go to new sensor
    }
  }
  // debugging();
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

// TODO: need to figure out velocity tolerance ???
void wait_for_movement() {
  // Note: we can also just use the motionDetected parameter from the PMW3360 library
  while (measVel[axis][sensorSelect] <= vel_tolerance) {
    doSensing();
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

    // Record time stamp of movement
    if (measVel[axis][sensorSelect] >= vel_tolerance) {
      timeLastGo = millis();
    }

    for (int i = 0; i < 4; i++) {
      // Sensor velocity sensing
      measVel[0][i] = -convTwosComp(data[i].dx);     // '-' convention is used to flip sensor's z axis
      measVel[1][i] = convTwosComp(data[i].dy);

      // Integrate linear velocities to get position
      estPos[0][i] = estPos[0][i] + measVel[0][i];
      estPos[1][i] = estPos[1][i] + measVel[1][i];
    }

    // Record time stamp of movement
    if (measVel[axis][sensorSelect] >= vel_tolerance) {
      timeLastGo = millis();
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

void writeToEEPROM() {
  // Write calibration values to EEPROM
  Serial.println("Write all values ('a') or just one sensor value (type sensor number)?");

  while (!Serial.available());
  char ch = Serial.read();

  // Handle serial input
  if (ch >= '0' && ch <= '3') {
    int index = ch - '0';         // convert char to int by subtracting ASCII value of '0'
    for (int j = 0; j < 2; j++) {
      eepromWrite(index, cVal[j][index]);
    }
  } else if (ch == 'a') {         // TO-DO: change to switch case if more inputs are needed
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 2; j++) {
        eepromWrite(i, cVal[j][i]);
      }
    }
  }
}

void eepromWrite(int address, float value) {
  // Write given calibration value to EEPROM in (0,1,2,3) format
  // Note: Each float value (32 bit) takes up 4 bytes, so the addresses
  //    for each sensor will be ((0,1,2,3),(4,5,6,7),(8,9,10,11),(12,13,14,15))
  int eeAddress = address * 4;

  // Convert the float to a byte array
  byte* floatAsBytes = (byte*)(void*)&value;

  // EEPROM.put(eeAddress, deviceID);
  // Write each byte of the float to EEPROM
  for (int i = 0; i < 4; i++) {
    EEPROM.write(eeAddress + i, floatAsBytes[i]);
  }
}

float eepromRead(int address) {
  // Read calibration value from given address (0,1,2,3)
  int eeAddress = address * 4;

  float value;
  byte* floatAsBytes = (byte*)(void*)&value;

  // Read each byte of the float from EEPROM
  for (int i = 0; i < 4; i++) {
    floatAsBytes[i] = EEPROM.read(eeAddress + i);
  }

  return value;
}

float meanArray(float* arr, int size) {
  // Calculate the mean of all values in an array
  if (size == 0) return 0; // Return 0 if the array is empty

  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += arr[i]; // Summing up all elements of the array
  }
  return sum / size; // Calculating the mean
}
