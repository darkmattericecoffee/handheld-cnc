// Libraries to include
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <PMW3360.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

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
void lineGenerator();
void sinGenerator();
void circleGenerator();
// Math functions
int16_t convTwosComp(int16_t value);
float myDist(float x1, float y1, float x2, float y2);
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
void ensureToolLowered();
void ensureToolRaised();
void raiseZ();
void lowerZ();
// Other loop functions
void sensorPlotting();
void debugging();
void outputSerial();
void parseNC(const char* filename);
void makePath();
void DesignModeToggle();

// Pin definitions -------------------------------------------------------------------------------
// Sensor pins
#define SS0   39   // Chip select pin. Connect this to SS on the module.
#define SS1   10
#define SS2   40
//#define SS3   32
int sensorPins[3] = {SS0, SS1, SS2};
// int sensorPins[4] = {SS0, SS1, SS2, SS3};
#define LIMIT_MACH_X0   8
#define LIMIT_MACH_Z0   2
#define BUTT_MACH_X0    4
#define BUTT_MACH_Z0    3
#define BUTT_WORK_X0Y0  6
#define BUTT_WORK_Z0    5
#define BUTT_HANDLE     7
#define POT_THICK       26

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

// Driver pins
#define SERIAL_PORT_X         Serial7     // HardwareSerial port
#define SERIAL_PORT_Z         Serial6

// Max path values
#define MAX_PATHS  4
#define MAX_POINTS 1000

// SD pins
const int chipSelect = BUILTIN_SDCARD;

// Constants ------------------------------------------------------------------------
// EEPROM addresses
const int eepromAddrCx = 0;  
const int eepromAddrCy = 12;  

// Modes
int plotting = 0;             // plot values  (1 = yes; 0 = no)
int debugMode = 1;            // print values (1 = yes; 0 = no)
int outputMode = 1;           // output data to serial
int designMode = 0;           // choose the design - from hardcode (line = 0; sine_wave = 1; circle = 2; gCode = 3)

// Path properties
int num_paths = 0;  // The actual number of paths
int num_points = 0; // The actual number of points
Point paths[MAX_PATHS][MAX_POINTS];

// Path properties (sine wave)
const float sinAmp = 5.0;
const float sinPeriod = 50.0;
const float pathMax_y = 300.0;            // x-length of entire path (mm) (used for line too)
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
long unsigned dt = 500;       // microseconds (freq = 1,000,000/timestepPoll [Hz])
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
// float maxVel = 6400.0;                // max velocity motor can move at (step/s)
float maxVel = 80.0*Conv;                // max velocity motor can move at (step/s)
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

// Variables ------------------------------------------------------------------------
// Calibration coeffs, these are variables since we set them by accessing eeprom in setup
// float Cx[3] = {0.00997506234f,0.01003310926f,0.00996611521f};
// float Cy[3] = {0.01011531459f,0.01026588646f,0.01019056354f};
float cVal[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                  {0.0f,0.0f,0.0f,0.0f}};

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

// Motor control variables
float desPos = 0.0f;              // desired position of tool (mm - 0 is center of gantry)
float desVel = 0.0f;              // desired velocity of tool (mm/s)

// Object Initialization ------------------------------------------------------------
// Sensor object creation
PMW3360 sensors[3];
//PMW3360 sensors[4];

// Motor object creation
AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);

// Driver objecet creation
TMC2209Stepper driverX(&SERIAL_PORT_X, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverZ(&SERIAL_PORT_Z, R_SENSE, DRIVER_ADDRESS);

// Router state
State state = POWER_ON;

// -------------------------------------------------------------------------------------------------
// Setup and Main Loop -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);  

  if (outputMode) {
    while(!Serial);
  }
  delay(100);         // as opposed to the while(!Serial);

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
  pinMode(BUTT_MACH_X0, INPUT);
  pinMode(BUTT_MACH_Z0, INPUT);
  pinMode(BUTT_WORK_X0Y0, INPUT);
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

  // Zero the machine X once on startup
  Serial.println("Waiting for machine X to be zeroed...");
  while (state != MACHINE_X_ZERO) {
    if(digitalRead(BUTT_MACH_X0) == LOW) {
      machineZeroX();
      state = MACHINE_X_ZERO;
    }
  }
  Serial.println("Machine x zeroed!");

  // Zero the workspace Z once on startup
  Serial.println("Waiting for workspace Z to be zeroed...");
  while (state != WORKSPACE_Z_ZERO) {
    if(digitalRead(BUTT_WORK_Z0) == LOW) {
      workspaceZeroZ();
      state = WORKSPACE_Z_ZERO;
    }
  }
  Serial.println("Workspace Z zeroed!");

  // Pick a design
  DesignModeToggle();
}

void loop() {
  // Sensing
  if(micros() - timeLastPoll >= dt) {
    doSensing();
  }
  
  // Serial Interface
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd') {
      DesignModeToggle();
    }
  }
  
  // Workspace X and Y zeroing
  if (state == DESIGN_SELECTED && digitalRead(BUTT_WORK_X0Y0) == LOW) {
    workspaceZeroXY();
    state = READY;

    // Reset cutting path
    path_started = false;
    current_path_idx = 0;
    current_point_idx = 0;
  }

  // Break here until we are ready to cut
  if (state != READY) {
    return;
  }

  // Safety stuff
  if (digitalRead(LIMIT_MACH_X0) == LOW) {
    // If X carriage runs into X limit switch
    stopStepperX();
    raiseZ();
    Serial.println("X limit reached");

    // Reset back to design mode
    DesignModeToggle();
  }

  if (digitalRead(LIMIT_MACH_Z0) == LOW) {
    // If Z carriage runs into Z limit switch
    stopStepperZ();
    stopStepperX();
    Serial.println("Z limit reached");

    // Reset back to design mode
    DesignModeToggle();
  }
  
  // Debugging
  if (debugMode) {
    debugging();
  }

  // Path logging
  if (outputMode) {
    outputSerial();
  }

  ///////////////////////////////////
  // START OF ACTUAL CUTTING LOGIC //
  ///////////////////////////////////
  Point goal = paths[current_path_idx][current_point_idx];
  Point next = paths[current_path_idx][current_point_idx + 1];

  // If we have not started the path, and the first point is behind us
  // keep the tool raised and return. We wait here until the first point
  // is in front of us and ready to be cut
  if (!path_started && signedDist(estPos[0], estPos[1], goal.x, goal.y, estYaw) > 0) {
    Serial.println("Cutting path starts behind router current position");

    // Move bit closest to intersect with cutting path
    desPos = desPosClosestToIntersect(estPos[0], estPos[1], estYaw, goal.x, goal.y, next.x, next.y);
    stepperX.moveTo(Conv*desPos);
    if (stepperX.distanceToGo() != 0) {
      stepperX.run();
    }

    return;
  }

  // If we get here start the path
  path_started = true;

  // Desired position if we intersect
  float desPos = desPosIntersect(estPos[0], estPos[1], estYaw, goal.x, goal.y, next.x, next.y);
  // Desired position if we do not intersect
  float desPosClosest = desPosClosestToIntersect(estPos[0], estPos[1], estYaw, goal.x, goal.y, next.x, next.y);

  bool handle_buttons_pressed = digitalRead(BUTT_HANDLE) == LOW;
  bool gantry_intersects = desPos != NAN;
  bool goal_behind_router = signedDist(estPos[0], estPos[1], goal.x, goal.y, estYaw) > 0;
  bool gantry_angle_ok = angleFrom(goal, next) < PI / 4;

  if (handle_buttons_pressed && gantry_intersects && goal_behind_router && gantry_angle_ok) {
    // We are good to cut
    ensureToolLowered();
    stepperX.moveTo(Conv*desPos);
    if (stepperX.distanceToGo() != 0) {
      stepperX.run();
    }

    // Update point index if needed
    if (signedDist(estPos[0], estPos[1], next.x, next.y, estYaw) > 0) {
      // If next point is behind router, it becomes the new goal.
      current_point_idx += 1;

      // If we're at the end of the points, stop cutting so we can start the next path
      if (current_point_idx == num_points-1) {
        Serial.println("Current path finished.");
        ensureToolRaised();
        current_point_idx = 0;
        current_path_idx += 1;

        // If we're done all paths then go back to design mode.
        if (current_path_idx == num_paths) {
          Serial.println("All paths finished.");
          DesignModeToggle();
        }
      }
    }
  } else {
    // Stop cutting
    ensureToolRaised();
    stepperX.moveTo(Conv*desPosClosest);
    if (stepperX.distanceToGo() != 0) {
      stepperX.run();
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Setup subfunctions -----------------------------------------------------------------------------
void sensorSetup() {
  // Sensor initialization
  for (int i = 0; i < ns; i++) {
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

void motorSetup() {
  // Set up motors
  // Initialize pins
  // pinMode(MS1_X, OUTPUT);
  // pinMode(MS2_X, OUTPUT);
  pinMode(MS1_Z, OUTPUT);
  pinMode(MS2_Z, OUTPUT);
  pinMode(MOT_EN_X, OUTPUT);
  pinMode(MOT_EN_Z, OUTPUT);

  // Initialize microstep
  // digitalWrite(MS1_X, LOW);
  // digitalWrite(MS2_X, LOW);
  digitalWrite(MS1_Z, LOW);
  digitalWrite(MS2_Z, LOW);

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
}

void doubleLineGenerator() {
  // One line going up at x = -5 and one line going down at x = 5
  for (int i=0; i<MAX_POINTS; i++) {
    float scale = (float)i / num_points - 1;
    paths[0][i] = Point{x: -20.0, y: pathMax_y * scale};
    paths[1][i] = Point{x: 20.0, y: pathMax_y * (1 - scale)};
  }

  num_paths = 2;
  num_points = MAX_POINTS;
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

void circleGenerator() {
  // Generate a circle path to cut
  float radius = circleDiameter / 2.0;
  float angle_step = 2.0 * PI / MAX_POINTS;

  for (int i = 0; i < MAX_POINTS; i++) {
    float angle = i * angle_step;
    float x = -radius + (radius * cosf(angle));
    float y = radius * sinf(angle);
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
    if (pCount < numPLine) {
      float y = (pathMax_y) * (float)i / (MAX_POINTS - 1);
      float x = std::fmod(y, zigSize);
      if (x > (zigSize / 2)) {
        x = zigSize - x;
      }
      
      paths[0][i] = Point{x, y};
    }
  }

  num_paths = 1;
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
    Serial.println("Lines are parallel, no intersection point.");
    return NAN;
  }

  float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
  float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
  
  // Check if the intersection point is on the gantry.
  if (t < 0 || t > 1) {
    Serial.println("Gantry does not intersect the line segment.");
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
    Serial.println("Lines are parallel, no intersection point.");
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
  float maxPos = gantryLength - xBuffer;

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
  for (int i = 0; i < ns; i++) {
    data[i] = sensors[i].readBurst();
  }

  // TODO: check all of the sensors and account for misreads

  // Sensor velocity sensing
  for (int i = 0; i < ns; i++) {
    // Sensor velocity sensing
    measVel[0][i] = -convTwosComp(data[i].dx)*cVal[0][i]/dt;     // '-' convention is used to flip sensor's z axis
    measVel[1][i] = convTwosComp(data[i].dy)*cVal[1][i]/dt;
  }

  // Body angle estimation
  estAngVel[0] = (measVel[0][2] - measVel[0][0])/ly;
  estAngVel[1] = (measVel[0][2] - measVel[0][1])/ly;
  estAngVel[2] = (measVel[1][1] - measVel[1][0])/lx;
  estAngVel[3] = (measVel[1][1] - measVel[1][2])/lx;
  // Simple average of angular velocities
  float sumAngVel = 0.0f;
  for (int i = 0; i<4; i++) {
    sumAngVel = sumAngVel + estAngVel[i];
  }
  estAngVel1 = sumAngVel / 4.0f;
  // Integrate angular velocity to get angle
  estYaw = estYaw + estAngVel1*dt;

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
  estPos[0] = estPos[0] + estVel1[0]*dt;
  estPos[1] = estPos[1] + estVel1[1]*dt;

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
  stepperX.setMaxSpeed(maxVel);
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
  stepperZ.setMaxSpeed(speed_x0);
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

void ensureToolLowered() {
  if (toolRaised) {
    lowerZ();
  }
}

void ensureToolRaised() {
  if (!toolRaised) {
    raiseZ();
  }
}

void raiseZ() {
  stepperZ.moveTo(Conv*restHeight);
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }

  toolRaised = 1;
}

void lowerZ() {
  // Lower tool

  int sensorVal = analogRead(POT_THICK);
  matThickness = mapF(sensorVal, 0, 1024, 0, maxThickness);

  stepperZ.moveTo(-Conv*matThickness);
  while (abs(stepperZ.distanceToGo()) > (Conv*1)) {
    stepperZ.run();
  }
  stepperZ.setMaxSpeed(speed_x1);             // initial speed faster now
  while (stepperZ.distanceToGo() !=0) {
    stepperZ.run();
  }
  stepperZ.setMaxSpeed(speed_x0);
  
  toolRaised = 0;
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
    
    Serial.printf("x:%f,y:%f,theta:%f",estPos[0],estPos[1],estYaw);
    Serial.println();
    // Serial.printf("x:%f,y:%f,theta:%f,xg:%f,yg:%f,desPos:%f",estPosX,estPosY,estYaw,goalX,goalY,desPos);
    Serial.printf("x_raw:%f,y_raw:%f",measVel[0][0],measVel[1][0]);
    // Serial.printf("thickness:%f, analog: %i",Conv*analogRead(POT_THICK), analogRead(POT_THICK));
    // Serial.printf("x:%f,y:%f,goalX:%f,goalY:%f,desPos:%i",estPosToolX,estPosToolY,goalX,goalY,desPos);
    // Serial.print(motorPosX);

    Serial.println();
  }
}

void outputSerial() {
  if(millis() - timeLastOutput >= dtOutput) {
    timeLastOutput = millis();

    // Starting label
    Serial.print("POS:");

    // Data to output
    Serial.printf("%f,%f,%f,%f,%f",estPos[0],estPos[1],estYaw,estPosTool[0],estPosTool[1]);
    Serial.printf(",%f,%f,%f",goalX,goalY,cutStarted);

    Serial.println();
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
      circleGenerator();
      Serial.println("Circle path generated!");
      break;
    case 3:
      parseNC("generic_test02.nc");
      break;
    case 4:
      parseNC("stupid_wave02.nc");
      break;
    case 5:
      zigZagGenerator();
      break;
  }

  for (int i=0; i < num_paths; i++) {
    for (int j = 0; j < num_points; j++) {
        // TODO: do for any size pathArray
        Serial.printf("PATH %d:%f,%f\n", i, paths[i][j].x, paths[i][j].y);
      }
  }
}

void DesignModeToggle() {
  Serial.println("Start Design Mode Toggle");
  
  // Clear the serial buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  while (!Serial.available()) {
    // Wait until there is data available in the serial buffer
  }
  
  int receivedNum = Serial.parseInt();
  
  Serial.print("Received num: ");
  Serial.println(receivedNum);
  
  designMode = receivedNum;
  
  makePath();
  
  Serial.println("End of Design Mode Toggle!");

  state = DESIGN_SELECTED;
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