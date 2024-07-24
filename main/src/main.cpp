// Libraries to include
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <PMW3360.h>
#include <PMW3360_SPI1.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <EncoderButton.h>
#include <fonts/FreeMonoBold12pt7b.h>
#include <fonts/FreeMonoBold9pt7b.h>
#include <fonts/FreeMono12pt7b.h>
#include <fonts/FreeMono9pt7b.h>

/*
Sensor configuration (USING 3 SENSORS NOW!!):
0 --- 1              ^ y
|  x  |   (old x <-) |-> x
2 --- (3 gone)
    NOTE: old config didn't obey the right hand rule. We are now flipping x.

Angle signage: +CCW
*/

// State values
typedef enum State {
  POWER_ON,
  MACHINE_X_ZERO,
  WORKSPACE_Z_ZERO,
  SETTING_THICKNESS,
  THICKNESS_SET,
  SELECTING_DESIGN,
  DESIGN_SELECTED,
  READY
} State;

// Coordinate
typedef struct Point {
  float x;
  float y;
} Point;

// Function definitions
// Setup functions
void sensorSetup();
void motorSetup();
void driverSetup();

// Path functions
void lineGenerator();
void sinGenerator();
void circleGenerator();
void squareGenerator();

// Math functions
int16_t convTwosComp(int16_t value);
float myDist(float x1, float y1, float x2, float y2);
float clamp(float val, float min, float max);
float principalAngleRad(float x);
float signedDist(float xr, float yr, float xg, float yg, float th);
float angleFrom(Point a, Point b);
float desPosIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4);
float desPosClosestToIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4);
float desiredPosition(float dX,float dY,float theta);
float mapF(long x, float in_min, float in_max, float out_min, float out_max);
void readEepromCalibration(float (&cVal)[2][4]);

// Sensing functions
void doSensing();

// Motor control functions
void enableStepperZ();
void disableStepperZ();
void stopStepperX();
void stopStepperZ();
void machineZeroX();
void workspaceZeroZ();
void workspaceZeroXY();

// Other loop functions
void sensorPlotting();
void debugging();
void outputSerial(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting);
void parseNC(const char* filename);
void makePath();
void encoderSetThickness();
void encoderDesignMode();

void drawShape();
void drawCenteredText(const char* text, int size);
void drawFixedUI();
void drawUI(Point goal, Point next, uint8_t i);
void drawDirection();

void calibrate();

// Encoder functions
void nullHandler(EncoderButton &eb);
void onClickResetState(EncoderButton &eb);
void onClickZeroMachineX(EncoderButton &eb);
void onClickZeroWorkspaceZ(EncoderButton &eb);
void onClickSetThickness(EncoderButton &eb);
void onEncoderSetThickness(EncoderButton &eb);
void onEncoderUpdateDesignMode(EncoderButton &eb);
void onClickMakePath(EncoderButton &eb);
void onClickZeroWorkspaceXY(EncoderButton &eb);

// Pin definitions -------------------------------------------------------------------------------
// Sensor pins
#define SS0   39   // Chip select pin. Connect this to SS on the module.
#define SS1   10
#define SS2   40
//#define SS3   32
int sensorPins[3] = {SS0, SS1, SS2};
// int sensorPins[4] = {SS0, SS1, SS2, SS3};
#define LIMIT_MACH_X0       8
#define LIMIT_MACH_Z0       2
#define BUTT_HANDLE         7
#define ENCODER_PIN_A       21
#define ENCODER_PIN_B       22
#define ENCODER_BUTTON_PIN  3

// Motor Pins
#define MS1_X       17
#define MS2_X       16
#define MOT_EN_X    18
#define MOT_DIR_X   14
#define MOT_STEP_X  15
#define MS1_Z       37
#define MS2_Z       38
#define MOT_EN_Z    41
#define MOT_DIR_Z   36
#define MOT_STEP_Z  33

// LCD Pins
#define TFT_CS     20
#define TFT_DC     19

// Driver pins
#define SERIAL_PORT_X         Serial7     // HardwareSerial port
#define SERIAL_PORT_Z         Serial6

// Max path values
#define MAX_PATHS  4
#define MAX_POINTS 1000

#define NUM_DESIGNS 6

// Webwork green is 0x88FF88 or RGB (136,255,136)
// The adafruit color mapping is a bit weird, and outlined here
// https://learn.adafruit.com/adafruit-gfx-graphics-library?view=all
// 
// Here are the converted values into the adafruit mapping
// r: 136/255 * 31 = 17 -> 0b10001
// g: 255/255 * 63 = 63 -> 0b111111
// b: 136/255 * 31 = 17 -> 0b10001
//
// Putting that together, the color code is 
// 0b1000111111110001 -> 0x8FF1
// 
#define GC9A01A_WEBWORK_GREEN 0x8FF1

// SD pins
const int chipSelect = BUILTIN_SDCARD;

// Constants ------------------------------------------------------------------------
// EEPROM addresses
const int eepromAddrCx = 0;  
const int eepromAddrCy = 12;  

// Modes
int plotting = 0;             // plot values  (1 = yes; 0 = no)
int debugMode = 1;            // print values (1 = yes; 0 = no)
int outputMode = 0;           // output data to serial
int designMode = 0;           // choose the design 

// Path properties
int num_paths = 0;  // The actual number of paths
int num_points = 0; // The actual number of points
Point paths[MAX_PATHS][MAX_POINTS];
int pathDir[MAX_PATHS] = {1,1,1,1};

// Path properties (sine wave)
const float sinAmp = 5.0;
const float sinPeriod = 50.0;
const float pathMax_y = 100.0;            // x-length of entire path (mm) (used for line too)
// Path properties (circle)
const float circleDiameter = 800.0;       // Diameter of the circle

// Material properties
float matThickness = 0.0;                   // thickness of material
float maxThickness = 10.0;                // upper bound of thickness knob (mm)
float restHeight = 4.0;                   // rest height of tool before cutting

// Timing constants
long unsigned debounceDelay = 50;      // the debounce time; increase if the output flickers
long unsigned dtDebug = 500;                   // (ms)
long unsigned dtOutput = 20;            // (ms)

// Sensor properties
const int ns = 3;                   // number of sensors
const int CPI = 2500;               // This value changes calibration coefficients
long unsigned dt = 900;       // microseconds (freq = 1,000,000/timestepPoll [Hz])
const float lx = 120.0f;                // x length of rectangular sensor configuration
const float ly = 140.0f;                // y length of rectangular sensor configuration
const float xOff[3] = {-lx/2,lx/2,-lx/2};
const float yOff[3] = {ly/2,ly/2,-ly/2};

// Motor properties
// Note: Constants are in units (steps/*) whereas variables are (mm/*). Kind of
// confusing, but whenedver the variables are used within an accelStepper function,
// multiply by Conv. This can be made simpler by using SpeedyStepper library.
int uSteps = 4;                       // microstep configuration
int Conv = 25*uSteps;                 // conversion factor (mm -> steps)
float stepPulseWidth = 20.0;          // min pulse width (from Mark Rober's code)
float maxCurrent_RMS = 640.0;         // motor RMS current rating (mOhm)
float maxSpeedX = 80.0*Conv;                // max velocity X motor can move at (step/s)
float maxSpeedZ = 20.0*Conv;                // max velocity Z motor can move at (step/s)
float maxAccel = 1600.0*Conv;             // max acceleration (step/s^2)
float retract = 5;                    // distance to retract (mm)
float speed_x0 = 20.0 * Conv;             // x zeroing speed (step/s)
float speed_x1 = 4.0 * Conv;              // x secondary zeroing speed (step/s)
float accel_x0 = 200.0 * Conv;            // x zeroing acceleration (step/s^2)
#define motorInterfaceType 1

// Driver properties
#define DRIVER_ADDRESS      0b00    // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE             0.11f   // Match to your driver
int TPWMTHRS = 0x753;                  // threshold velocity for spreadCycle (at 1 mm/s right now)

// Gantry geometry
float gantryLength = 106.0;         // usable length of x-gantry (mm)
float xLimitOffset = 2.54;          // distance from wall of stepper when zeroed (mm)
float xBuffer = 3.0;                // safety buffer between tool body and walls (mm)
float zLength = 34.0;               // usable length of z-gantry (mm)
float zLimitOffset = 2.13;          // distance from wall when zeroed (mm)
float maxHeight = zLength;          // max height that z can actuate without collision
                                    // (really depends on zeroed position)

int16_t radius, centerX, centerY, x2, y2 = 0;

// Variables ------------------------------------------------------------------------
// Calibration coeffs, these are variables since we set them by accessing eeprom in setup
// float Cx[3] = {0.00997506234f,0.01003310926f,0.00996611521f};
// float Cy[3] = {0.01011531459f,0.01026588646f,0.01019056354f};
float cVal[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                  {0.0f,0.0f,0.0f,0.0f}};
// (1.000000, 1.000000) 1: (0.997606, 1.004717) 2: (0.989168, 1.000850)
float selfCal[2][3] = {{1.0f,0.997606f,0.989168f},
                      {1.0f,1.004717f,1.000850f}};

// Run states
int limitHitX = 0;
int limitHitZ = 0;
int cutStarted = 0;
int toolRaised = 1;

// Button states
int readyOrNot = 0;                 // determines whether system is zeroed 
                                    // (0 = workpiece is unzeroed; 1 = z is zeroed; 2 = xy is zeroed)

// Zeroing variables
int x0_count = 2;           // x zeroing count variable (start as "false")
int z0_count = 2;           // z zeroing count variable (start as "false")

// Timing variables
int firstPoint = 1;
long unsigned timeLastPoll = 0;
long unsigned timeLastOutput = 0;
long unsigned timeLastDebug = 0;
long unsigned timeLastDebounce = 0;   // (ms)
long unsigned sensingTime = 0;        // (us)

// Measured quantities
float measVel[2][3] = {{0.0f,0.0f,0.0f},
                      {0.0f,0.0f,0.0f}};     // BFF (x,y) velocity of each sensor (mm/us)

// Estimated quantities
float estVel[2][3] = {{0.0f,0.0f,0.0f},
                    {0.0f,0.0f,0.0f}};      // world frame (x,y) velocity of router (mm/us)
float estVel1[2] = {0.0f,0.0f};              // averaged (x,y) velocity calculation (mm/us)
float estPos[2] = {0.0f,0.0f};                // router center position (x,y)
float estPosTool[2] = {0.0f,0.0f};            // tool center position (x,y)
float estAngVel[4] = {0.0f,0.0f,0.0f,0.0f};   // angular velocity of router (rad/s) (4 vals from 3 sensors)
float estAngVel1 = 0.0f;                      // averaged velocity calculation (rad/s)
float estYaw = 0.0f;                          // orientation of router (rad)
float estTraj = 0.0f;                     // (..?) trajectory angle wrt. world frame (n. direction of velocity. Best way to track this?)
float estVelAbs = 0.0f;                   // (unsure why used..?) absolute velocity

// Motor quantities
float motorPosX = 0.0f;                       // motor position (in mm!!)
float motorPosZ = 0.0f;                       // motor position (in mm!!)

// Control Constants ----------------------------------------------------------------
const int num_queries = 2;        // number of points to query for min distance calculation

// Control Variables ----------------------------------------------------------------
// Path following
int current_path_idx = 0;
int current_point_idx = 0;
bool path_started = false;

float goalX = 0.0f;               // goal point x coordinate (mm)
float goalY = 0.0f;               // goal point y coordinate (mm)
float prevX = 0.0f;               // previous point x coordinate (mm)
float prevY = 0.0f;               // previous point y coordinate (mm)

long lastDraw = 0;

uint8_t iter = 0;

int16_t lastX0, lastY0, lastX1, lastY1, lastX2, lastY2, lastX3, lastY3, lastTargetCircleX, lastTargetCircleY;

// Motor control variables

// Object Initialization ------------------------------------------------------------
// Sensor object creation
PMW3360 sensors[2];
PMW3360_SPI1 sensors_SPI1[1];
//PMW3360 sensors[4];

// Motor object creation
AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);

// Driver objecet creation
TMC2209Stepper driverX(&SERIAL_PORT_X, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverZ(&SERIAL_PORT_Z, R_SENSE, DRIVER_ADDRESS);

// Router state
State state = POWER_ON;
bool cutting = false; // TODO: delete me
bool prevChecks[4]; // TODO: delete me

EncoderButton encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_BUTTON_PIN);
// Adafruit_GC9A01A screen = Adafruit_GC9A01A(TFT_CS, TFT_DC);
Adafruit_GC9A01A screen = Adafruit_GC9A01A(&SPI1, TFT_DC, TFT_CS);

void nullHandler(EncoderButton &eb) {
  Serial.println("null handler called");
  return; // Don't do anything
}

void onClickGoToDesignMode(EncoderButton &eb) {
  encoderDesignMode();
}

void onClickResetState(EncoderButton &eb) {
  drawCenteredText("Zero Machine X", 1);
  state = POWER_ON;
  encoder.setClickHandler(onClickZeroMachineX);
}

void onClickZeroMachineX(EncoderButton &eb) {
  drawCenteredText("Zeroing Machine X...", 1);
  machineZeroX();
  state = MACHINE_X_ZERO;
  drawCenteredText("Zero Workspace Z", 1);
  encoder.setClickHandler(onClickZeroWorkspaceZ);
}

void onClickZeroWorkspaceZ(EncoderButton &eb) {
  drawCenteredText("Zeroing Workspace Z...", 1);
  workspaceZeroZ();
  state = WORKSPACE_Z_ZERO;
  encoderSetThickness();
}

void onClickZeroWorkspaceXY(EncoderButton &eb) {
  drawCenteredText("Zeroing Workspace XY...", 1);
  workspaceZeroXY();

  // Reset cutting path
  path_started = false;
  current_path_idx = 0;
  current_point_idx = 0;

  state = READY;
  // drawFixedUI();
  encoder.setClickHandler(nullHandler);
}

void onEncoderUpdateThickness(EncoderButton &eb) {
  // NUM_DESIGNS to have correct wrapping for negative numbers
  float incrScalar = 0.1;
  float tempThickness = matThickness + eb.increment()*incrScalar;

  if (tempThickness <= maxThickness && tempThickness >= 0) {
    matThickness = tempThickness;
  }
  
  char text2send[50]; // Define a buffer large enough to hold the formatted string
  sprintf(text2send, "Turn to set thickness\n%.2f mm", matThickness);
  drawCenteredText(text2send, 1);
  // Serial.println(matThickness);
}

void onClickSetThickness(EncoderButton &eb) {
  state = THICKNESS_SET;
}

void encoderSetThickness() {
  char text2send[50]; // Define a buffer large enough to hold the formatted string
  sprintf(text2send, "Turn to set thickness\n%.2f mm", matThickness);
  drawCenteredText(text2send, 1);

  state = SETTING_THICKNESS;
  encoder.setEncoderHandler(onEncoderUpdateThickness);
  encoder.setClickHandler(onClickSetThickness);

  while (state != THICKNESS_SET) {
    encoder.update();
  }

  encoderDesignMode();
}

void onEncoderUpdateDesignMode(EncoderButton &eb) {
  // NUM_DESIGNS to have correct wrapping for negative numbers
  designMode = (NUM_DESIGNS + designMode + eb.increment()) % NUM_DESIGNS;
  drawShape();
}

void onClickMakePath(EncoderButton &eb) {
  makePath();
  state = DESIGN_SELECTED;
}

void encoderDesignMode() {
  drawShape();

  state = SELECTING_DESIGN;
  encoder.setEncoderHandler(onEncoderUpdateDesignMode);
  encoder.setClickHandler(onClickMakePath);

  while (state != DESIGN_SELECTED) {
    encoder.update();
  }

  // Hack for opensauce, auto-zero XY
  workspaceZeroXY();

  // Reset cutting path
  path_started = false;
  current_path_idx = 0;
  current_point_idx = 0;

  state = READY;
  encoder.setEncoderHandler(nullHandler);
  encoder.setClickHandler(nullHandler);

  screen.fillScreen(GC9A01A_BLACK);
  drawDirection();

  // Clear out sensors in case we moved while in design mode
  sensors[0].readBurst();
  sensors[1].readBurst();
  sensors_SPI1[0].readBurst();

  // TODO: uncomment to add in manual XY zeroing
  // drawCenteredText("Zero Workspace XY", 1);
  // encoder.setEncoderHandler(nullHandler);
  // encoder.setClickHandler(onClickZeroWorkspaceXY);
}

// -------------------------------------------------------------------------------------------------
// Setup and Main Loop -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);  

  if (outputMode) {
    while(!Serial);
  }
  delay(100);         // as opposed to the while(!Serial);

  // Initialize the display
  screen.begin();
  screen.fillScreen(GC9A01A_BLACK);

  radius = screen.width()*0.95 / 4;
  centerX = screen.width() / 2;
  centerY = screen.width() / 2;

  drawCenteredText("Initializing...", 1);
  // screen.setRotation(1);

  // Load calibration coeffs
  Serial.println("Loading calibration coefficients:");
  readEepromCalibration(cVal);

  Serial.print("Cx values: ");
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < ns; i++) {
      Serial.print(cVal[j][i], 4);
      if (i < ns - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
    if (j == 0){
      Serial.print("Cy values: ");
    }
  }

  // Limit switch initialization
  pinMode(LIMIT_MACH_X0, INPUT);
  pinMode(LIMIT_MACH_Z0, INPUT);

  // Button initialization
  pinMode(BUTT_HANDLE, INPUT);
  //handleButton.setDebounceTime(50);

  // Setup systems
  sensorSetup();

  motorSetup(); 

  driverSetup();
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  // calibrate();

  encoder.setClickHandler(onClickZeroMachineX);
  encoder.setTripleClickHandler(onClickGoToDesignMode);
  onClickResetState(encoder);
}

void loop() {  
  // Sensing
  if(micros() - timeLastPoll >= dt) {
    sensingTime = micros() - timeLastPoll;
    doSensing();
  }

  // Run steppers
  stepperX.run();
  stepperZ.run();
  encoder.update();

  // Serial Interface
  // if (Serial.available()) {
  //   char ch = Serial.read();
  //   if (ch == 'd') {
  //     DesignModeToggle();
  //   }
  // }
  
  // Workspace X and Y zeroing
  // if (state == DESIGN_SELECTED && digitalRead(ENCODER_BUTTON_PIN) == LOW) {
  //   workspaceZeroXY();
  //   state = READY;

  //   // Reset cutting path
  //   path_started = false;
  //   current_path_idx = 0;
  //   current_point_idx = 0;
  // }

  // Break here until we are ready to cut
  if (state != READY) {
    return;
  }

  // Safety stuff
  if (digitalRead(LIMIT_MACH_X0) == LOW) {
    // If X carriage runs into X limit switch
    stopStepperX();
    stepperZ.moveTo(Conv*restHeight);
    while (stepperZ.distanceToGo() != 0) {
      stepperZ.run();
    }
    Serial.println("X limit reached");

    // Reset back to design mode
    encoderDesignMode();
  }

  if (digitalRead(LIMIT_MACH_Z0) == LOW) {
    // If Z carriage runs into Z limit switch
    stopStepperZ();
    stopStepperX();
    Serial.println("Z limit reached");

    // Reset back to design mode
    encoderDesignMode();
  }
  
  // Debugging
  if (debugMode) {
    debugging();
  }

  ///////////////////////////////////
  // START OF ACTUAL CUTTING LOGIC //
  ///////////////////////////////////
  Point goal = paths[current_path_idx][current_point_idx];
  Point next = paths[current_path_idx][current_point_idx + 1];
  
  // TODO: removed this for the opensauce UI. Add back if you want.
  // if ((millis()-lastDraw) > 15) {
  //   iter = (iter + 1)%7;
  //   // unsigned long now = micros();
  //   drawUI(goal, next, iter);
  //   // Serial.printf("draw %d took %i us\n", iter, micros()-now);
  //   lastDraw = millis();
  // }
  

  // If we have not started the path, and the first point is behind us
  // keep the tool raised and return. We wait here until the first point
  // is in front of us and ready to be cut
  if (!path_started && pathDir[current_path_idx] * signedDist(estPos[0], estPos[1], goal.x, goal.y, estYaw) > 0) {
    // Serial.println("Cutting path starts behind router current position");

    // Move bit closest to intersect with cutting path
    float desPos = desPosClosestToIntersect(estPos[0], estPos[1], estYaw, goal.x, goal.y, next.x, next.y);

    if (outputMode) {
      outputSerial(estPos[0], estPos[1], estYaw, goal, stepperX.currentPosition()/Conv, desPos, false);
    }

    stepperX.moveTo(Conv*desPos);

    return;
  }

  // If we get here start the path
  path_started = true;

  // Desired position if we intersect
  float desPos = desPosIntersect(estPos[0], estPos[1], estYaw, goal.x, goal.y, next.x, next.y);
  // Desired position if we do not intersect
  float desPosClosest = desPosClosestToIntersect(estPos[0], estPos[1], estYaw, goal.x, goal.y, next.x, next.y);

  // Conditions for cutting
  bool handle_buttons_pressed = digitalRead(BUTT_HANDLE) == LOW;
  bool handle_buttons_debounce = (millis() - timeLastDebounce) < debounceDelay;
  if (handle_buttons_pressed) { timeLastDebounce = millis(); }
  bool handle_buttons_ok = handle_buttons_pressed || handle_buttons_debounce;
  bool gantry_intersects = desPos != NAN;
  bool goal_behind_router = pathDir[current_path_idx] * signedDist(estPos[0], estPos[1], goal.x, goal.y, estYaw) > 0;
  bool gantry_angle_ok = angleFrom(goal, next) > (PI / 6);

  // TODO: delete me
  bool newChecks[4] = {handle_buttons_ok, gantry_intersects, goal_behind_router, gantry_angle_ok};
  bool logged = false;
  for (int i=0; i<4; i++) {
    if (prevChecks[i] != newChecks[i] && !logged) {
      Serial.println("Checks have changed:");
      Serial.printf("\thandle_buttons_ok: %d\n", handle_buttons_ok);
      Serial.printf("\tgantry_intersects: %d\n", gantry_intersects);
      Serial.printf("\tgoal_behind_router: %d\n", goal_behind_router);
      Serial.printf("\tgantry_angle_ok: %d\n", gantry_angle_ok);

      logged = true;
    }

    prevChecks[i] = newChecks[i];
  }

  if (handle_buttons_ok && gantry_intersects && goal_behind_router && gantry_angle_ok) {
    // Path logging
    if (outputMode) {
      outputSerial(estPos[0], estPos[1], estYaw, goal, stepperX.currentPosition()/Conv, desPos, true);
    }

    // if (!cutting) {
    //   Serial.println("Cutting");
    //   cutting = true;
    // }

    // We are good to cut
    stepperZ.moveTo(-Conv*matThickness);
    stepperX.moveTo(Conv*desPos);

    // Update point index if needed
    if (pathDir[current_path_idx] * signedDist(estPos[0], estPos[1], next.x, next.y, estYaw) > 0) {
      // If next point is behind router, it becomes the new goal.
      current_point_idx += 1;
      // Serial.println("Moving to next point");

      // If we're at the end of the points, stop cutting so we can start the next path
      if (current_point_idx == num_points-1) {
        Serial.println("Current path finished");
        stepperZ.moveTo(Conv*restHeight);
        path_started = false;
        current_point_idx = 0;
        current_path_idx += 1;

        // If we're done all paths then go back to design mode.
        if (current_path_idx == num_paths) {
          while (stepperZ.distanceToGo() != 0) {
            stepperZ.run();           // make sure tool is raised after path is finished
          }
          Serial.println("All paths finished");
          encoderDesignMode();
        } else {
          drawDirection();
        }
      }
    }
  } else {
    // Path logging
    if (outputMode) {
      outputSerial(estPos[0], estPos[1], estYaw, goal, stepperX.currentPosition()/Conv, desPosClosest, false);
    }

    // if (cutting) {
    //   Serial.println("Cutting Stopped");
    //   cutting = false;
    // }
    
    // Stop cutting
    stepperZ.moveTo(Conv*restHeight);
    stepperX.moveTo(Conv*desPosClosest);
  }
}

// ------------------------------------------------------------------------------------------------
// Setup subfunctions -----------------------------------------------------------------------------
void sensorSetup() {
  // Sensor initialization
  for (int i = 0; i < 2; i++) {
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
  if (sensors_SPI1[0].begin(sensorPins[2], CPI)) {
    Serial.print("Sensor2");
    Serial.print(" initialization succeeded, with CPI = ");
    Serial.println(sensors_SPI1[0].getCPI());
  } else {
    Serial.print("Sensor");
    Serial.print("2");
    Serial.println(" initialization failed");
  }
}

void motorSetup() {
  // Set up motors
  // Initialize pins
  // pinMode(MS1_X, OUTPUT);
  // pinMode(MS2_X, OUTPUT);
  // pinMode(MS1_Z, OUTPUT);
  // pinMode(MS2_Z, OUTPUT);
  pinMode(MOT_EN_X, OUTPUT);
  pinMode(MOT_EN_Z, OUTPUT);

  // Initialize microstep
  // digitalWrite(MS1_X, LOW);
  // digitalWrite(MS2_X, LOW);
  // digitalWrite(MS1_Z, LOW);
  // digitalWrite(MS2_Z, LOW);

  // Enable motors (Mark Rober disables steppers initially..?)
  digitalWrite(MOT_EN_X, LOW);
  digitalWrite(MOT_EN_Z, LOW);

  // Disable motors
  delay(100);
  disableStepperZ();        // so that tool can be moved down manually to work surface
  //digitalWrite(MOT_EN_X, HIGH);
  //digitalWrite(MOT_EN_Z, HIGH);

  // Set motor properties
  stepperX.setMinPulseWidth(stepPulseWidth);
  stepperX.setMaxSpeed(speed_x0);
  stepperX.setAcceleration(maxAccel);
  stepperX.setCurrentPosition(0);
  stepperZ.setMinPulseWidth(stepPulseWidth);
  stepperZ.setMaxSpeed(speed_x0);
  stepperZ.setAcceleration(maxAccel);
  stepperZ.setCurrentPosition(0);
}

void driverSetup() {
  // Set up TMC2209 drivers
  SERIAL_PORT_X.begin(115200);      // INITIALIZE UART TMC2209
  SERIAL_PORT_Z.begin(115200);
  delay(100);

  driverX.begin();                // Initialize driver                   
  driverX.toff(5);                // Enables driver in software
  driverZ.begin();
  driverZ.toff(5);

  driverX.rms_current(maxCurrent_RMS);       // Set motor RMS current
  driverZ.rms_current(maxCurrent_RMS);       // Set motor RMS current
  if (uSteps > 1) {
    driverX.microsteps(uSteps);               // Set microsteps
    driverZ.microsteps(uSteps);               // Set microsteps
  } else {
    driverX.microsteps(0);
    driverZ.microsteps(0);
  }

  driverX.pwm_autoscale(true);    // Needed for stealthChop
  driverX.en_spreadCycle(false);   // false = StealthChop / true = SpreadCycle
  driverX.TPWMTHRS(TPWMTHRS);     // enable hybrid mode with velocity threshold
  driverZ.pwm_autoscale(true);    // Needed for stealthChop
  driverZ.en_spreadCycle(false);   // false = StealthChop / true = SpreadCycle
  driverX.TPWMTHRS(TPWMTHRS);     // enable hybrid mode with velocity threshold
}

void lineGenerator() {
  // Generate line path to cut
  for (int i = 0; i < MAX_POINTS; i++) {
    paths[0][i] = Point{x: 0, y: (pathMax_y) * (float)i / (MAX_POINTS - 1)};
  }

  num_paths = 1;
  num_points = MAX_POINTS;
}

void sinGenerator() {
  // Generate sine path to cut
  for (int i = 0; i < MAX_POINTS; ++i) {
    float y = (pathMax_y) * (float)i / (MAX_POINTS - 1);
    float x = sinAmp * sinf((TWO_PI/sinPeriod)*y);
    paths[0][i] = Point{x, y};
  }

  num_paths = 1;
  num_points = MAX_POINTS;
}

void zigZagGenerator() {
  // Generate a zig zaxg to cut
  int numPLine = 50;
  int pCount = 0;
  // int leftRight = 1;
  float zigSize = 40;

  for (int i = 0; i < MAX_POINTS; ++i) {

    float y = (pathMax_y) * (float)i / (MAX_POINTS - 1);
    float x = std::fmod(y, zigSize);
    if (x > (zigSize / 2)) {
      x = zigSize - x;
    }
    
    // Serial.printf("Points (%f, %f) \n ", x, y);
    paths[0][i] = Point{x, y};

  }

  
  num_paths = 1;
  num_points = MAX_POINTS;
}

void doubleLineGenerator() {
  // One line going up at x = -20 and one line going down at x = 20
  float length = 100.0;

  for (int i=0; i<MAX_POINTS; i++) {
    float scale = (float)i / (MAX_POINTS - 1);
    paths[0][i] = Point{x: -20.0, y: length * scale};
    paths[1][i] = Point{x: 20.0, y: length * (1 - scale)};
  }

  pathDir[0] = 1;
  pathDir[1] = -1;

  num_paths = 2;
  num_points = MAX_POINTS;
}

// Makes a 4 path circle with the given configuration:
void circleGenerator() {
  float r = 30.0;
  Point center = Point{x: 0.0, y: 50.0};
  float theta;
  
  // Path alternates forward and backward
  pathDir[0] = 1;
  pathDir[1] = -1;
  pathDir[2] = 1;
  pathDir[3] = -1;

  for (int i=0; i<MAX_POINTS; i++) {
    theta = (float)i/MAX_POINTS*PI/2;
    
    // Q4: 270->360
    paths[0][i] = Point{
      x: center.x + r*cosf(3*PI/2 + theta), 
      y: center.y + r*sinf(3*PI/2 + theta)
    };

    // Q2: 90->180
    paths[1][i] = Point{
      x: center.x + r*cosf(PI/2 + theta), 
      y: center.y + r*sinf(PI/2 + theta)
    };

    // Q3: 270->180
    paths[2][i] = Point{
      x: center.x + r*cosf(3*PI/2 - theta), 
      y: center.y + r*sinf(3*PI/2 - theta)
    };

    // Q1: 90->0 
    paths[3][i] = Point{
      x: center.x + r*cosf(PI/2 - theta), 
      y: center.y + r*sinf(PI/2 - theta)
    };
  }

  num_paths = 4;
  num_points = MAX_POINTS;
}

void squareGenerator() {
  float angle = 60;
  float angle_rad = angle * (M_PI / 180.0);
  float segment_length = 100.0;
  pathDir[0] = 1;
  pathDir[1] = -1;

  // Calculate the x and y increments based on the angle
  float y_increment = segment_length / (MAX_POINTS - 1);
  float x_increment = y_increment / tan(angle_rad);

  // Generate diamond path to cut
  for (int p = 0; p < 2; p++) {
    for (int i = 0; i < MAX_POINTS; i++) {
      int xIndex = (i >= MAX_POINTS / 2) ? (MAX_POINTS - 1 - i) : i;
      int yIndex = p == 1 ? (MAX_POINTS - 1 - i) : i;
      paths[p][i] = Point{x: pathDir[p] * xIndex * x_increment, y: yIndex * y_increment};
    }
  }

  num_paths = 2;
  num_points = MAX_POINTS;
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

float myDist(float x1, float y1, float x2, float y2) {
  // Calculate the distance between two points

  return sqrt(pow(x1 - x2,2) + pow(y1 - y2,2));
}

float clamp(float val, float min, float max) {
  if (val < min) {
    return min;
  }

  if (val > max) {
    return max;
  }

  return val;
}

// Returns x mapped between 0 and PI. This assumes that
// the line is infinite, so angle may be 180 degrees
// different from the original value x.
// Ex: 3 PI/2 -> PI/2
float principalAngleRad(float x) {
  while (x > PI) {
    x -= PI;
  }

  while (x < 0) {
    x += PI;
  }

  return x;
}

float signedDist(float xr, float yr, float xg, float yg, float th) {
  // Calculate the signed distance between goal point and line of gantry
  // Note: if the distance is:
  //    < 0 - the point is in front of the gantry (it is yet to be passed)
  //    > 0 - the point is behind the gantry (it has been passed)

  float m = tan(th);
  float b = yr - m*xr;
  float A = m;
  float B = -1;
  float C = b;

  return (A*xg + B*yg + C)/sqrt(pow(A,2) + pow(B,2));
}

// Returns the angle (rads) between the gantry and the 
// line connecting points a and b.
float angleFrom(Point a, Point b) {
  float th1 = principalAngleRad(atan2f(b.y-a.y, b.x-a.x));

  // Using yaw here is a bit odd, since yaw is 0 when the router is in the
  // original orientation, which would actually be 90 degrees from +x. However,
  // we actually care about the angle of the gantry, which will be 0 degrees from +x
  // when the yaw is 0, so this works ok.
  float th2 = principalAngleRad(estYaw);

  return abs(th1-th2);
}

// desPosIntersect returns the desired stepperX position such that the 
// cutting tool intersects the infinite line from (x3,y3) to (x4,y4).
// If the gantry does not intersect the line, this returns NAN.
// (x,y) is the current position of the router, and th is the yaw.
float desPosIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4) {
  // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
  float x1 = xc - (cosf(th)*gantryLength/2);
  float y1 = yc - (sinf(th)*gantryLength/2);
  float x2 = xc + (cosf(th)*gantryLength/2);
  float y2 = yc + (sinf(th)*gantryLength/2);
  float den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

  // Check for parallel lines (denominator is zero)
  if (den == 0) {
    return NAN;
  }

  float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
  float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
  
  // Check if the intersection point is on the gantry.
  if (t < 0 || t > 1) {
    return NAN;
  }
  
  float x = x1 + t * (x2 - x1);
  float y = y1 + t * (y2 - y1);

  float dx = x - xc;
  float dy = y - yc;

  return dx*cosf(th) + dy*sinf(th);
}

// desPosIntersect returns the desired stepperX position such that the 
// cutting tool intersects the infinite line from (x3,y3) to (x4,y4).
// If the gantry does not intersect the line, this returns the position
// that gets the tool closest to intersecting the line.
// (x,y) is the current position of the router, and th is the yaw.
float desPosClosestToIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4) {
  // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
  float x1 = xc - (cosf(th)*gantryLength/2);
  float y1 = yc - (sinf(th)*gantryLength/2);
  float x2 = xc + (cosf(th)*gantryLength/2);
  float y2 = yc + (sinf(th)*gantryLength/2);
  float den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

  // Check for parallel lines (denominator is zero)
  if (den == 0) {
    // If lines are parallel, just keep the stepper where it is
    return stepperX.currentPosition() / Conv;
  }

  float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
  float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
  
  float x = x1 + t * (x2 - x1);
  float y = y1 + t * (y2 - y1);

  float dx = x - xc;
  float dy = y - yc;

  float desiredPos = dx*cosf(th) + dy*sinf(th);
  float maxPos = (gantryLength / 2.0) - xBuffer;

  return clamp(desiredPos, -maxPos, maxPos);
}

float desiredPosition(float dX,float dY,float theta) {
  // Calculate the desired position for the tool

  float desPosition = (dX + tanf(theta)*dY)*cosf(theta);       // Sanzhar equation
  // this can also just be written:
  // float desPosition = dX*cosf(th) + dY*sinf(th);
  
  return desPosition;
}

float mapF(long x, float in_min, float in_max, float out_min, float out_max) {
  // Maps a float value

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Sensing functions
void doSensing() {
  //Serial.println(micros() - timeLastPoll);
  timeLastPoll = micros();

  // Sensing ---------------------------------------------------------------------
  // Collect sensor data (raw)
  PMW3360_DATA data[ns];
  for (int i = 0; i < 2; i++) {
    data[i] = sensors[i].readBurst();
  }
  data[2] = sensors_SPI1[0].readBurst();

  // TODO: check all of the sensors and account for misreads

  // Sensor velocity sensing
  for (int i = 0; i < ns; i++) {
    // Sensor velocity sensing
    measVel[0][i] = -convTwosComp(data[i].dx)*cVal[0][i]/sensingTime;     // '-' convention is used to flip sensor's z axis
    measVel[1][i] = convTwosComp(data[i].dy)*cVal[1][i]/sensingTime;
  }

  // Body angle estimation
  estAngVel[0] = (measVel[0][2] - measVel[0][0])/ly;
  estAngVel[1] = (measVel[0][2] - measVel[0][1])/ly;
  estAngVel[2] = (measVel[1][1] - measVel[1][0])/lx;
  estAngVel[3] = (measVel[1][1] - measVel[1][2])/lx;
  // TODO: filter out bad sensor measurements
  //    - compare angular velocities and see if any are a certain threshold or standard deviation off of average
  //    - compare measVel redings that are colinear (they should be equal)
  //    - determine if any sensors appear repeatedly erroneous for these checks. If so, throw out their readings
  // Simple average of angular velocities
  float sumAngVel = 0.0f;
  for (int i = 0; i<4; i++) {
    sumAngVel = sumAngVel + estAngVel[i];
  }

  estAngVel1 = sumAngVel / 4.0f;
  // Integrate angular velocity to get angle
  estYaw = estYaw + estAngVel1*sensingTime;

  // Body position estimation
  // TODO: simplify with matrix operation for rotation (using for loops)
  estVel[0][0] = measVel[0][0]*cosf(estYaw)-measVel[1][0]*sinf(estYaw) + 0.5*estAngVel1*(lx*cosf(estYaw)-ly*sinf(estYaw));
  estVel[0][1] = measVel[0][1]*cosf(estYaw)-measVel[1][1]*sinf(estYaw) + 0.5*estAngVel1*(lx*cosf(estYaw)+ly*sinf(estYaw));
  estVel[0][2] = measVel[0][2]*cosf(estYaw)-measVel[1][2]*sinf(estYaw) + 0.5*estAngVel1*(-lx*cosf(estYaw)-ly*sinf(estYaw));
  estVel[1][0] = measVel[0][0]*sinf(estYaw)+measVel[1][0]*cosf(estYaw) + 0.5*estAngVel1*(ly*cosf(estYaw)+lx*sinf(estYaw));
  estVel[1][1] = measVel[0][1]*sinf(estYaw)+measVel[1][1]*cosf(estYaw) + 0.5*estAngVel1*(-ly*cosf(estYaw)+lx*sinf(estYaw));
  estVel[1][2] = measVel[0][2]*sinf(estYaw)+measVel[1][2]*cosf(estYaw) + 0.5*estAngVel1*(ly*cosf(estYaw)-lx*sinf(estYaw));
  // Simple average of linear velocities
  float sumVelX = 0.0f;
  float sumVelY = 0.0f;
  for (int i = 0; i<ns; i++) {
    sumVelX = sumVelX + estVel[0][i];
    sumVelY = sumVelY + estVel[1][i];
  }
  estVel1[0] = sumVelX / ns;
  estVel1[1] = sumVelY / ns;
  // Integrate linear velocities to get position
  estPos[0] = estPos[0] + estVel1[0]*sensingTime;
  estPos[1] = estPos[1] + estVel1[1]*sensingTime;

  // Additional values
  estTraj = atanf(estVel1[1]/estVel1[0]);    // trajectory angle w.r.t inertial frame
  estVelAbs = sqrt(pow(estVel1[0],2) + pow(estVel1[1],2));

  // Sensor plotting
  if (plotting) {
    sensorPlotting();
  }
}

// Motor control functions
void enableStepperZ() {
  digitalWrite(MOT_EN_Z, LOW);
}

void disableStepperZ() {
  // stepperZ.disableOutputs();
  digitalWrite(MOT_EN_Z, HIGH);
}

void stopStepperX() {
  // Stop stepper motor (not using acceleration. If acceleration is desired, user library's stop() function)
  stepperX.setSpeed(0);
  stepperX.runSpeed();
}

void stopStepperZ() {
  // Stop stepper motor (not using acceleration. If acceleration is desired, user library's stop() function)
  stepperZ.setSpeed(0);
  stepperZ.runSpeed();
}

void machineZeroX() {
  // Run until we hit the xlimit
  stepperX.setSpeed(speed_x0);
  while (digitalRead(LIMIT_MACH_X0) == HIGH) {
    stepperX.runSpeed();
  }

  // Back off
  stepperX.move(-Conv*retract);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  // Second, slower pass
  stepperX.setSpeed(speed_x1);
  while (digitalRead(LIMIT_MACH_X0) == HIGH) {
    stepperX.runSpeed();
  }

  // Move to the center
  stepperX.move(-Conv*((gantryLength/2) - xLimitOffset));
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  // Set current position as 0
  stepperX.setCurrentPosition(0);

  // Reset speed and accel settings
  stepperX.setMaxSpeed(maxSpeedX);
  stepperX.setAcceleration(maxAccel);
}

void workspaceZeroZ() {
  // Manually set workpiece Z. This is initiated only after 
  // the user has manually moved the tool to the top of the surface.
  enableStepperZ();

  // Set the starting position as 0, the user manually set this
  stepperZ.setCurrentPosition(0);

  // Initial pass
  stepperZ.setSpeed(speed_x0);
  while (digitalRead(LIMIT_MACH_Z0) == HIGH) {
    stepperZ.runSpeed();
  }

  // Retract
  stepperZ.move(-Conv*retract);
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }

  // Second slow pass
  stepperZ.setSpeed(speed_x1);
  while (digitalRead(LIMIT_MACH_Z0) == HIGH) {
    stepperZ.runSpeed();
  }

  // Set this as the max height
  maxHeight = stepperZ.currentPosition() / Conv;

  // Return to rest height
  stepperZ.moveTo(Conv*restHeight);
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }

  // Reset max speed
  stepperZ.setMaxSpeed(maxSpeedZ);
}

void workspaceZeroXY() {
  // Make sure tool is centered
  stepperX.moveTo(0);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  // Make sure tool is at rest height
  stepperZ.moveTo(Conv*restHeight);
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }

  // Set all working position and orientation data to 0
  estPos[0] = 0;
  estPos[1] = 0;
  estYaw = 0;
  estPosTool[0] = 0;
  estPosTool[1] = 0;
}

void sensorPlotting() {
  // Plot sensor data

  //Serial.printf("dx:%f,dy:%f",measVelX,measVelY);
  //Serial.printf("dx:%i,dy:%i",data.dx,data.dy);
  Serial.printf("x:%f,y:%f,theta:%f",estPos[0],estPos[1],estYaw);
//      Serial.printf("w1:%f,w2:%f,w3:%f,w4:%f,w5:%f,w6:%f,w7:%f,w8:%f",estAngVel[0],estAngVel[1],
//        estAngVel[2],estAngVel[3],estAngVel[4],estAngVel[5],estAngVel[6],estAngVel[7]);
  //Serial.printf("x:%f,y:%f",xmm[1],ymm[1]);
  Serial.println();
}

void debugging() {
  if(millis() - timeLastDebug >= dtDebug) {
    timeLastDebug = millis();
    // Print debug data
    // Put all Serial print lines here to view
    
    Serial.printf("x:%f,y:%f,theta:%f\n",estPos[0],estPos[1],estYaw);
    // Serial.printf("S0 | x: %f, y: %f\n", measVel[0][0], measVel[1][0]);
    // Serial.printf("S1 | x: %f, y: %f\n", measVel[0][1], measVel[1][1]);
    // Serial.printf("S2 | x: %f, y: %f\n", measVel[0][2], measVel[1][2]);
    // Serial.printf("w0:%f,w0:%f,w0:%f,%w0:%f\n", 1000*estAngVel[0], 1000*estAngVel[1], 1000*estAngVel[2], 1000*estAngVel[3]);
    // Serial.printf("x:%f,y:%f,theta:%f,xg:%f,yg:%f,desPos:%f",estPosX,estPosY,estYaw,goalX,goalY,desPos);
    // Serial.printf("x_raw:%f,y_raw:%f\n",measVel[0][0],measVel[1][0]);
    // Serial.printf("thickness:%f, analog: %i",Conv*analogRead(POT_THICK), analogRead(POT_THICK));
    // Serial.printf("x:%f,y:%f,goalX:%f,goalY:%f,desPos:%i",estPosToolX,estPosToolY,goalX,goalY,desPos);
    // Serial.print(motorPosX);
    // Serial.printf("des_pos:%f,z_stepper_pos:%f\n",desPos,measVel[1][0]);
    // Serial.printf("curr_pnt_idx:%i,curr_path_idx:%i\n",current_point_idx, current_path_idx);
    // Serial.printf("Sensing time = %i\n", sensingTime);

  }
}

// toolPos and desPos in mm
void outputSerial(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting) {
  if(millis() - timeLastOutput >= dtOutput) {
    timeLastOutput = millis();

    float toolX = estX + toolPos*cosf(estYaw);
    float toolY = estY + toolPos*sinf(estYaw);

    float desX = estX + desPos*cosf(estYaw);
    float desY = estY + desPos*sinf(estYaw);

    Serial.printf(
      "POS:%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
      estX,
      estY,
      estYaw,
      goal.x,
      goal.y,
      toolX,
      toolY,
      desX,
      desY,
      cutting
    );
  }
}

void parseNC(const char* filename) {
  // NC gCode parsing function
  File myFile = SD.open(filename);
  if (!myFile) {
    Serial.println("Failed to open file for reading");
    return;
  }

  int idx = 0;
  while (myFile.available()) {
    String line = myFile.readStringUntil('\n');
    int xPos = line.indexOf('X');
    int yPos = line.indexOf('Y');
    if (xPos != -1 && yPos != -1) {
      int spacePos = line.indexOf(' ', xPos);
      // if (spacePos == -1) {
      //   spacePos = line.length();
      // }
      float x = line.substring(xPos+1, spacePos).toFloat();

      spacePos = line.indexOf(' ', yPos);
      if (spacePos == -1) {
        spacePos = line.length();
      }
      float y = line.substring(yPos+1, spacePos).toFloat();

      paths[0][idx] = Point{x,y};

      idx++;        // only go to update index if there is a valid XY coordinate
    }
  }

  myFile.close();
}

void makePath() {
  // Make path
  switch (designMode) {
    case 0:
      lineGenerator();
      Serial.println("Line path generated!");
      break;
    case 1:
      sinGenerator();
      Serial.println("Sine wave path generated!");
      break;
    case 2:
      zigZagGenerator();
      Serial.println("Zig-zag path generated!");
      break;
    case 3:
      doubleLineGenerator();
      Serial.println("Double line path generated!");
      break;
    case 4:
      squareGenerator();
      Serial.println("Circle path generated!");
      break;
    case 5:
      circleGenerator();
      Serial.println("Circle path generated!");
      break;
  }

  for (int i=0; i < num_paths; i++) {
    for (int j = 0; j < num_points; j++) {
        // TODO: do for any size pathArray
        Serial.printf("PATH:%d,%f,%f\n", i, paths[i][j].x, paths[i][j].y);
      }
  }
}

void readEepromCalibration(float (&cVal)[2][4]) {
  // int addr = eepromAddrCx;
  // for (int i = 0; i < ns; i++) {
  //   Cx[i] = EEPROM.get(addr, Cx[i]);
  //   addr += sizeof(float);
  // }

  // addr = eepromAddrCy;
  // for (int i = 0; i < ns; i++) {
  //   Cy[i] = EEPROM.get(addr, Cy[i]);
  //   addr += sizeof(float);
  // }

  int addr = 0;
  for (int i = 0; i < ns; i++) {
    for (int j = 0; j < 2; j++) {
      EEPROM.get(addr, cVal[j][i]);
      addr += sizeof(float);
    }
  }
}

void calibrate(){

  float calDistance = 100.0;

  float calPos[2][3] = {{0.0f, 0.0f, 0.0f},
                    {0.0f,0.0f,0.0f}};

// do for each axis
  for (int axis = 0; axis <2; axis++){
    delay(500);
    calPos[axis][0] = 0.0;

    screen.fillScreen(GC9A01A_BLACK);

    Serial.println("about to sprintf");
    char *msg;
    // sprintf(msg, "Move %f mm in %d direction", calDistance, axis);
    drawCenteredText("Move router in direction", 1);
    Serial.printf("Move router in %i direction", axis);

    // Serial.println("sprintf worked!");
    // move calDistance cm in given axis direction
    while (calPos[axis][0]<calDistance){
      if(micros() - timeLastPoll >= dt) {
        doSensing();
        for (int i = 0; i < ns; i++){
          calPos[axis][i] = calPos[axis][i] + estVel[axis][i]*dt;
        }
      }

      Serial.println(calPos[axis][0]);
    }
    
    screen.fillScreen(GC9A01A_BLACK);

    // Serial.println("STOP");
    drawCenteredText("STOP",  1);
    delay(100);
    // set calibration coefficients based on sensor0
    for (int i = 0; i<ns; i++){
      selfCal[axis][i] = calPos[axis][0]/calPos[axis][i];
    }    
  }

  Serial.println("Calibration values:");
  Serial.printf("\t0: (%f, %f)", selfCal[0][0], selfCal[1][0]);
  Serial.printf("\t1: (%f, %f)", selfCal[0][1], selfCal[1][1]);
  Serial.printf("\t2: (%f, %f)", selfCal[0][2], selfCal[1][2]);

}

void drawShape() {
  int16_t tftWidth = screen.width();
  int16_t tftHeight = screen.height();
  int16_t centerX = tftWidth / 2;
  int16_t centerY = tftHeight / 2;
  int16_t size = min(tftWidth, tftHeight) / 3;

  float scale;
  int16_t minY,maxY,minX,maxX,y_quarter,y_3_quarter,x;

  screen.fillScreen(GC9A01A_BLACK);

  switch (designMode) {
    case 0:
      // line
      screen.drawLine(centerX, centerY-size, centerX, centerY+size, GC9A01A_WHITE);
      break;
    case 1:
      // sin
      scale = size / PI;
      for (int y = -size; y <= size; y++) {
        x = (int16_t) (scale*sin(y/scale));
        screen.drawPixel(centerX+x, centerY+y, GC9A01A_WHITE);
      }
      break;
    case 2:
      // zigzag
      minY = centerY - size;
      maxY = centerY + size;
      minX = centerX - size / 2;
      maxX = centerX + size / 2;

      y_quarter = minY + size / 2;
      y_3_quarter = maxY - size / 2;

      screen.drawLine(centerX, minY, maxX, y_quarter, GC9A01A_WHITE);
      screen.drawLine(maxX, y_quarter, minX, y_3_quarter, GC9A01A_WHITE);
      screen.drawLine(minX, y_3_quarter, centerX, maxY, GC9A01A_WHITE);

      break;
    case 3:
      // double line
      screen.drawLine(centerX-size/4, centerY-size, centerX-size/4, centerY+size, GC9A01A_WHITE);
      screen.drawLine(centerX+size/4, centerY-size, centerX+size/4, centerY+size, GC9A01A_WHITE);
      break;
    case 4:
      // diamond
      screen.drawLine(centerX-size, centerY, centerX, centerY+size, GC9A01A_WHITE);
      screen.drawLine(centerX, centerY+size, centerX+size, centerY, GC9A01A_WHITE);
      screen.drawLine(centerX+size, centerY, centerX, centerY-size, GC9A01A_WHITE);
      screen.drawLine(centerX, centerY-size, centerX-size, centerY, GC9A01A_WHITE);
      break;
    case 5:
      // circle
      screen.drawCircle(centerX, centerY, size, GC9A01A_WHITE);
      break;
  }
}

void drawCenteredText(const char* text, int size) {
  Serial.print("SCREEN: ");
  Serial.println(text);

  screen.fillScreen(GC9A01A_BLACK);

  int16_t tftWidth = screen.width();
  int16_t tftHeight = screen.height();
  int16_t centerX = tftWidth / 2;
  int16_t centerY = tftHeight / 2;

  screen.setFont(&FreeMonoBold9pt7b);
  screen.setTextSize(size);
  screen.setTextColor(GC9A01A_WHITE);

  // Split the text into lines based on '\n'
  char *line = strtok(text, "\n");
  int lineCount = 0;
  char *lines[10];  // Assuming a maximum of 10 lines for simplicity

  while (line != NULL) {
    lines[lineCount++] = line;
    line = strtok(NULL, "\n");
  }
  Serial.println(lineCount);

  // Calculate the total height of the text block
  int16_t totalHeight = lineCount * size * 10; // Approximate height per line based on text size
  int16_t yStart = centerY - totalHeight / 2;

  for (int i = 0; i < lineCount; i++) {
    int16_t x1, y1;
    uint16_t w, h;
    screen.getTextBounds(lines[i], 0, 0, &x1, &y1, &w, &h);

    // Calculate the top-left corner to start the text so that it gets centered
    int16_t xStart = centerX - w / 2;

    screen.setCursor(xStart, yStart + i * size * 10);
    screen.println(lines[i]);
  }
}

void drawDirection() {
  float width = 30;
  float height = screen.height() / 3;
  float spacing = 5;

  int16_t centerX = screen.height() / 2;
  int16_t centerY = screen.height() / 2;
  
  // Forward triangle
  int16_t x0 = centerX - width;
  int16_t y0 = centerY-spacing;
  int16_t x1 = centerX;
  int16_t y1 = centerY-height;
  int16_t x2 = centerX + width;
  int16_t y2 = centerY-spacing;

  // Backward triangle
  int16_t x3 = centerX - width;
  int16_t y3 = centerY+spacing;
  int16_t x4 = centerX;
  int16_t y4 = centerY+height;
  int16_t x5 = centerX + width;
  int16_t y5 = centerY+spacing;

  uint16_t forwardColor, reverseColor;

  if (pathDir[current_path_idx] > 0) {
    forwardColor = GC9A01A_WEBWORK_GREEN;
    reverseColor = GC9A01A_DARKGREY;
  } else {
    forwardColor = GC9A01A_DARKGREY;
    reverseColor = GC9A01A_WEBWORK_GREEN;
  }

  screen.drawTriangle(x0,y0,x1,y1,x2,y2,forwardColor);
  screen.drawTriangle(x3,y3,x4,y4,x5,y5,reverseColor);
}

void drawFixedUI() {
  screen.fillScreen(GC9A01A_BLACK);

  int16_t radius = screen.width()*0.95 / 2;
  int16_t centerX = screen.width() / 2;
  int16_t centerY = screen.width() / 2;

  // Draw the arcs on the edge of the screen
  for (int i=0; i<radius; i++) {
    float xOffset = radius*cosf(PI/6*i/radius);
    float yOffset = radius*sinf(PI/6*i/radius);

    screen.drawPixel(centerX - xOffset, centerY - yOffset, GC9A01A_WHITE);
    screen.drawPixel(centerX - xOffset, centerY + yOffset, GC9A01A_WHITE);
    screen.drawPixel(centerX + xOffset, centerY - yOffset, GC9A01A_WHITE);
    screen.drawPixel(centerX + xOffset, centerY + yOffset, GC9A01A_WHITE);
  }
}

void drawUI(Point goal, Point next, uint8_t i) {
  int16_t radius = screen.width()*0.95 / 4;
  int16_t centerX = screen.width() / 2;
  int16_t centerY = screen.width() / 2;

  float dTheta = estYaw + PI/2 - atan2f(next.y-goal.y, next.x-goal.x);

  float theta = estYaw - atan2f(goal.y-estPos[1], goal.x-estPos[0]);
  float dist = myDist(estPos[0], estPos[1], goal.x, goal.y);

  // Serial.printf("yaw: %f\n", degrees(estYaw));

  float offsetRadius = radius*0.8*tanh(dist*0.1);

  switch (i%7) {
    case 0:
      // Draw the target circle in the center
      screen.drawLine(centerX, centerY-5, centerX, centerY+5, GC9A01A_WHITE);
      screen.drawLine(centerX-5, centerY, centerX+5, centerY, GC9A01A_WHITE);
      break;
    case 1:
      // clear the old line left
      screen.drawLine(lastX0, lastY0, lastX1, lastY1, GC9A01A_BLACK);
      break;
    case 2:
      // clear old line right
      screen.drawLine(lastX2, lastY2, lastX3, lastY3, GC9A01A_BLACK);
      break;
    case 3:
      // draw the new line left
      lastX0 = centerX + 0.6*radius*cosf(dTheta);
      lastY0 = centerY + 0.6*radius*sinf(dTheta);
      lastX1 = centerX + radius*cosf(dTheta);
      lastY1 = centerY + radius*sinf(dTheta);

      screen.drawLine(lastX0, lastY0, lastX1, lastY1, GC9A01A_WHITE);
      break;
    case 4:
      lastX2 = centerX - radius*cosf(dTheta);
      lastY2 = centerY - radius*sinf(dTheta);
      lastX3 = centerX - 0.6*radius*cosf(dTheta);
      lastY3 = centerY - 0.6*radius*sinf(dTheta);
      
      screen.drawLine(lastX2, lastY2, lastX3, lastY3, GC9A01A_WHITE);
      break;
    case 5:
      // Clear the old target circle
      screen.drawCircle(lastTargetCircleX, lastTargetCircleY, 5, GC9A01A_BLACK);
      break;
    case 6:
      lastTargetCircleX = centerX + offsetRadius*cosf(theta);
      lastTargetCircleY = centerY + offsetRadius*sinf(theta);
      screen.drawCircle(lastTargetCircleX, lastTargetCircleY, 5, GC9A01A_WHITE);
      break;
  }
}