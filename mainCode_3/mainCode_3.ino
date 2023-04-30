// Libraries to include
#include <PMW3360.h>
#include <AccelStepper.h>

// Files to include
/*
Sensor configuration (USING 3 SENSORS NOW!!):
0 --- 1              ^ y
|  x  |   (old x <-) |-> x
2 --- (3 gone)
    NOTE: old config didn't obey the right hand rule. We are now flipping x.

Angle signage: +CCW
*/

// Pin definitions -------------------------------------------------------------------------------
// Sensor pins
#define SS0   39   // Chip select pin. Connect this to SS on the module.
#define SS1   10
#define SS2   40
//#define SS3   32
#define LIMIT_MACH_X0   8
#define LIMIT_MACH_Z0   2
#define BUTT_MACH_X0    4
#define BUTT_MACH_Z0    3
#define BUTT_WORK_X0Y0   6
#define BUTT_WORK_Z0   5
#define BUTT_HANDLE     7
#define POT_THICK   31

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

//int dirPin = 36;
//int stepPin = 33;
//#define RX2 0
//#define TX2 1
//#define DIAG_PIN 12

// Constants ------------------------------------------------------------------------
int plotting = 0;
int debugMode = 1;
int generalMode = 0;          // use general mode (sine wave) = 1; line drawing = 0;
int designMode = 1;           // 1 = sin wave; 2 = circle0
int cheatMode = 0;

// Path properties (sine wave)
const int num_points = 1000;             // length of path array
const float sinAmp = 5.0;
const float sinPeriod = 100.0;
const float pathMax_y = 300.0;
float pathArrayX[num_points];
float pathArrayY[num_points];
// Path properties (circle)
const float circleDiameter = 200.0;       // Diameter of the circle

// Button properties
long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Sensor properties
const int ns = 3;                   // number of sensors
const int CPI = 2500;               // This value changes calibration coefficients
long unsigned dt = 500;       // microseconds (freq = 1,000,000/timestepPoll [Hz])
const float Cx[3] = {0.00997506234f,0.01003310926f,0.00996611521f};
const float Cy[3] = {0.01011531459f,0.01026588646f,0.01019056354f};
const float lx = 120.0f;                // x length of rectangular sensor configuration
const float ly = 140.0f;                // y length of rectangular sensor configuration
const float xOff[3] = {-lx/2,lx/2,-lx/2};
const float yOff[3] = {ly/2,ly/2,-ly/2};

// Motor properties
int count = 0;
int uSteps = 8;
int Conv = 25*uSteps;
//int pos[] = {20,-20,20,-20};
//int vel[] = {10,-20,10,-40};
float stepPulseWidth = 20.0;          // min pulse width (from Mark Rober's code)
float maxVel = 40.0 * Conv;           // max velocity motor can move at
float maxAccel = 800.0 * Conv;
float retract = 5;                // (mm)
float speed_x0 = 20.0 * Conv;             // x zeroing speed (mm/s)
float speed_x1 = 4.0 * Conv;              // x secondary zeroing speed
float accel_x0 = 200.0 * Conv;
bool began = false;
#define motorInterfaceType 1

// Gantry geometry
float gantryLength = 106.0;       // (mm)
float xLimitOffset = 2.54;         // distance from wall of stepper when zeroed (mm)
float xBuffer = 3.0;
float zLength = 34.0;
float zLimitOffset = 2.13;
float maxHeight = zLength;

// Variables ------------------------------------------------------------------------
// Run states
int cutStarted = 0;
int raised = 0;

// Button states
int readyOrNot = 0;

// Zeroing variables
int x0_count = 2;           // x zeroing count variable (start as "false")
int z0_count = 2;           // z zeroing count variable (start as "false")

// Timing variables
int firstPoint = 1;
long dtDebug = 500;         // (ms)
long unsigned timeLastPoll = 0;
long unsigned timeLastDebug = 0;
long unsigned lastDebounceTime = 0;   // (ms)

// Measured quantities
int dX[3] = {0,0,0};
int dY[3] = {0,0,0};
float dXmm[3] = {0.0f,0.0f,0.0f};
float dYmm[3] = {0.0f,0.0f,0.0f};
float vXmm[3] = {0.0f,0.0f,0.0f};        // velocity (mm/s)(dXmm/dt) (unused)
float vYmm[3] = {0.0f,0.0f,0.0f};

// Estimated quantities
float estVelX[3] = {0.0f,0.0f,0.0f};
float estVelY[3] = {0.0f,0.0f,0.0f};
float estVelX1 = 0.0f;                    // (mm/us)
float estVelY1 = 0.0f;                    // (mm/us)
float estPosX = 0.0f;                         // frame center position - x
float estPosY = 0.0f;                         // frame center position - y
float estPosRoutX = 0.0f;                     // router center position - x
float estPosRoutY = 0.0f;                     // router center position - y
float estAngVel[4] = {0.0f,0.0f,0.0f,0.0f};
float estAngVel1 = 0.0f;
float estYaw = 0.0f;                          // Angle estimation
float estTraj = 0.0f;
float estVelAbs = 0.0f;

// Motor quantities
float motorPosX = 0.0f;
float motorPosZ = 0.0f;

// Control Constants ----------------------------------------------------------------
const int num_queries = 2;       // number of points to query for min distance calculation

// Control Variables ----------------------------------------------------------------
// Path following
int closest_point_index = 0;
float nextX = 0.0f;
float nextY = 0.0f;
float lastX = 0.0f;
float lastY = 0.0f;
float nextTrajC = 0.0f;           // trajectory in BFF from BFF y-axis (goes through BFF (0,0))

// Motor control variables
float desPos = 0.0f;
float desVel = 0.0f;

// Object Initialization ------------------------------------------------------------
// Sensor object creation
PMW3360 sensor0;
PMW3360 sensor1;
PMW3360 sensor2;
//PMW3360 sensor3;

// Motor object creation
AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);

// -------------------------------------------------------------------------------------------------
// Setup and Main Loop -----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);  
  while(!Serial);

  // Limit switch initialization
  pinMode(LIMIT_MACH_X0, INPUT);
  pinMode(LIMIT_MACH_Z0, INPUT);

  // Button initialization
  pinMode(BUTT_MACH_X0, INPUT);
  pinMode(BUTT_MACH_Z0, INPUT);
  pinMode(BUTT_WORK_X0Y0, INPUT);
  pinMode(BUTT_HANDLE, INPUT);
  //handleButton.setDebounceTime(50);

  sensorSetup();

  motorSetup();

  // Make path
  switch (designMode) {
    case 1:
      sinGenerator();
    case 2:
      circleGenerator();
  }
//  for (int i = 1; i < num_points; i++) {
//    Serial.printf("x(%i) = 
//  }

  Serial.println("Handheld CNC Router set up!");
  
  delay(500);
}

void loop() {
  // System Initialization ------------------------------------------------------------------------
  // Machine X Zeroing
  if (digitalRead(BUTT_MACH_X0) == LOW) {
    x0_count = 0;
  }
  if (x0_count == 0) {
    //Serial.println("Init limit switch");
    machineZeroX_1();
  } else if (x0_count == 1) {
    machineZeroX_2();
  }

  // Machine Z Zeroing
  if (digitalRead(BUTT_MACH_Z0) == LOW) {
    z0_count = 0;
    enableStepperZ();
    //Serial.println("z homing");
  }
  machineZeroZ();

  // Workpiece zeroing
  // X and Y
  if (digitalRead(BUTT_WORK_X0Y0) == LOW) {
    readyOrNot += 1;
  }
  // Z
  if (digitalRead(BUTT_WORK_Z0) == LOW) {
    z0_count = 0;
    workZeroZ_man();
    readyOrNot += 1;
  }

  // Sensing and Control ---------------------------------------------------------------------------
  if (readyOrNot > 1) {       // keep in mind this may occur premptively b/c of X0Y0 reading
    if(micros() - timeLastPoll >= dt) {
      //Serial.println(micros() - timeLastPoll);
      timeLastPoll = micros();
  
      // Sensing ---------------------------------------------------------------------
      // Collect sensor data
      PMW3360_DATA data0 = sensor0.readBurst_simple();
      PMW3360_DATA data1 = sensor1.readBurst_simple();
      PMW3360_DATA data2 = sensor2.readBurst_simple();
      //PMW3360_DATA data3 = sensor3.readBurst_simple();
  
      int yup = 1;
      if(yup) {
      //if(data1.isOnSurface && data1.isMotion) {   // If movement...
        // **Only checking sensor 1 rn (TODO: check all of them and account for misreads)**
  
        // Sensor velocity sensing
        dXmm[0] = -convTwosComp(data0.dx)*Cx[0] / dt;
        dXmm[1] = -convTwosComp(data1.dx)*Cx[1] / dt;
        dXmm[2] = -convTwosComp(data2.dx)*Cx[2] / dt;
        //dXmm[3] = -convTwosComp(data3.dx)*Cx[3] / dt;
        dYmm[0] = convTwosComp(data0.dy)*Cx[0] / dt;
        dYmm[1] = convTwosComp(data1.dy)*Cx[1] / dt;
        dYmm[2] = convTwosComp(data2.dy)*Cx[2] / dt;
        //dYmm[3] = convTwosComp(data3.dy)*Cx[3] / dt;
  
        // Body angle estimation
        estAngVel[0] = (dXmm[2] - dXmm[0])/ly;
        estAngVel[1] = (dXmm[2] - dXmm[1])/ly;
        estAngVel[2] = (dYmm[1] - dYmm[0])/lx;
        estAngVel[3] = (dYmm[1] - dYmm[2])/lx;
        // Simple average of angular velocities
        float sumAngVel = 0.0f;
        for (int i = 0; i<4; i++) {
          sumAngVel = sumAngVel + estAngVel[i];
          //Serial.printf("w%i:%f,",i,estAngVel[i]);
        }
        estAngVel1 = sumAngVel / 4.0f;
        estYaw = estYaw + estAngVel1*dt;
        //Serial.printf("w:%f",estAngVel1);
  
        // Body position estimation
        estVelX[0] = dXmm[0]*cosf(estYaw)-dYmm[0]*sinf(estYaw) + 0.5*estAngVel1*(lx*cosf(estYaw)-ly*sinf(estYaw));
        estVelX[1] = dXmm[1]*cosf(estYaw)-dYmm[1]*sinf(estYaw) + 0.5*estAngVel1*(lx*cosf(estYaw)+ly*sinf(estYaw));
        estVelX[2] = dXmm[2]*cosf(estYaw)-dYmm[2]*sinf(estYaw) + 0.5*estAngVel1*(-lx*cosf(estYaw)-ly*sinf(estYaw));
        estVelY[0] = dXmm[0]*sinf(estYaw)+dYmm[0]*cosf(estYaw) + 0.5*estAngVel1*(ly*cosf(estYaw)+lx*sinf(estYaw));
        estVelY[1] = dXmm[1]*sinf(estYaw)+dYmm[1]*cosf(estYaw) + 0.5*estAngVel1*(-ly*cosf(estYaw)+lx*sinf(estYaw));
        estVelY[2] = dXmm[2]*sinf(estYaw)+dYmm[2]*cosf(estYaw) + 0.5*estAngVel1*(ly*cosf(estYaw)-lx*sinf(estYaw));
        // Simple average of linear velocities
        float sumVelX = 0.0f;
        float sumVelY = 0.0f;
        for (int i = 0; i<ns; i++) {
          sumVelX = sumVelX + estVelX[i];
          sumVelY = sumVelY + estVelY[i];
        }
        estVelX1 = sumVelX / ns;              // velocity w.r.t. intertial frame
        estVelY1 = sumVelY / ns;              // velocity w.r.t. intertial frame
        estPosX = estPosX + estVelX1*dt;      // position w.r.t. intertial frame
        estPosY = estPosY + estVelY1*dt;      // position w.r.t. intertial frame
        estTraj = atanf(estVelY1/estVelX1);    // trajectory angle w.r.t inertial frame
        estVelAbs = sqrt(pow(estVelX1,2) + pow(estVelY1,2));
        //Serial.println(estTraj);
        
//        float absVel = sqrt(pow(dXmm[0],2) + pow(dYmm[0],2)) * (1000000.0);
//        Serial.printf("Absolute velocity = %f",absVel);
//        Serial.println();
  
        // Sensor plotting
        if (plotting) {
          sensorPlotting();
        }
      }
    }
  
    // Control ---------------------------------------------------------------------------------
    if (digitalRead(BUTT_HANDLE) == LOW  && closest_point_index + 1 < num_points &&
        abs(motorPosX/Conv) < (0.5*gantryLength - xBuffer)) {
      cutStarted = 1;
      lastDebounceTime = millis();
      //Serial.println(cutStarted);

      // Router position
      motorPosX = - stepperX.currentPosition() / (float)Conv;
      estPosRoutX = estPosX + motorPosX*cosf(estYaw);
      estPosRoutY = estPosY + motorPosX*cosf(estYaw);

      // Find closest point
      // TO-DO: find closest point by going to the next point in the array after crossing a point
      float min_distance = 1000;                  // ridiculously large initial min distance (mm)
      int start_point = closest_point_index;
      int end_point = min(start_point + num_queries, num_points);
      for (int i = start_point; i < end_point; ++i) {
        // float dist = distance(estPosX, estPosY, pathArrayX[i], pathArrayY[i]);
        float dist = myDist(estPosRoutX, estPosRoutY, pathArrayX[i], pathArrayY[i]);
        if (dist < min_distance) {
          min_distance = dist;
          closest_point_index = i;
        }
      }
  
      nextX = pathArrayX[closest_point_index];
      nextY = pathArrayY[closest_point_index];
      // nextTraj = atanf((nextY - estPosRoutY)/(nextX - estPosRoutX));
      if (nextX == estPosX) {
        nextTrajC = - estYaw;
      } else {
        nextTrajC = (PI/2) - atanf((nextY - estPosY)/(nextX - estPosX)) - estYaw;
      }
      float deltaX = nextX - estPosX;
      float deltaY = nextY - estPosY;
      
      // Determine desired actuation
      if (generalMode) {
        // Desired quantities (for general drawing)
        if (!cheatMode) {
          desPos = desiredPosition(deltaX,deltaY,estYaw);
          // desPos = (deltaX - tanf(estYaw)*deltaY)*cosf(estYaw);
        } else {
          desPos = sinAmp * sinf((TWO_PI/sinPeriod)*estPosY);     // cheat mode
        }
        if (!isnan(estTraj)) {
          desVel = estVelAbs * cosf(estTraj - estYaw) * 1000000;
        }
      } else {
        // Desired quantities (for line drawing)
        desPos = -estPosX*cosf(estYaw);
        if (!isnan(estTraj)) {
          desVel = estVelAbs * cosf(estTraj - estYaw) * 1000000;
        }
      }

      // Motor actuation ---------------------------------------------------------------------------
      stepperX.moveTo(Conv*desPos);
//        if (Conv*desVel <= maxVel) {
//          myStepper.setMaxSpeed(Conv*desVel);
//        }
      //myStepper.setMaxSpeed(maxVel);
      if (stepperX.distanceToGo() != 0) {
        //delay(100);
        stepperX.run();
      }

      // Save last point
      if (start_point != closest_point_index) {
        lastX = nextX;
        lastY = nextY;
      }
    }
    // React to non-operational state ---------------------------------------------------------------
    else if (cutStarted  && (millis() - lastDebounceTime) > debounceDelay) {
      // USER IS NOT IN CONTROL OF DEVICE -> cancel everything
      stopStepperX();
      //readyOrNot = 0;
      //Serial.println("User not in control!");

      // Z-up
      raiseZ();

      // Reset
      cutStarted = 0;

      Serial.println("User is no longer in control");
      debugMode = 0;
    }
    if (x0_count == 2 && digitalRead(LIMIT_MACH_X0) == LOW) {
      // If X carriage runs into X limit switch
      stopStepperX();
      raiseZ();
      cutStarted = 0;
      
      Serial.println("X limit reached");
      debugMode = 0;
    }
    if (z0_count == 2 && digitalRead(LIMIT_MACH_Z0) == LOW) {
      // If Z carriage runs into Z limit switch
      stopStepperZ();
      cutStarted = 0;
      
      Serial.println("Z limit reached");
      debugMode = 0;
    }
    
    // Debugging -----------------------------------------------------------------------------------
    if(millis() - timeLastPoll >= dtDebug) {
      timeLastDebug = millis();
      if (debugMode) {
        debugging();
      }
    }
    //delay(10);
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

//  // Initialize sensor locations relative to body
//  Serial.print("Initial locations of sensors: ");
//  for (int i = 0; i<ns; i++) {
//    xmm[i] = xmm[i] + xOff[i];
//    ymm[i] = ymm[i] + yOff[i];
//    Serial.printf("%i(%.2f,%.2f),",i+1,xmm[i],ymm[i]);
//  }
//  Serial.println();
}

void motorSetup() {
  // Setup motors
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  // Initialize pins
  pinMode(MS1_X, OUTPUT);
  pinMode(MS2_X, OUTPUT);
  pinMode(MS1_Z, OUTPUT);
  pinMode(MS2_Z, OUTPUT);
  pinMode(MOT_EN_X, OUTPUT);
  pinMode(MOT_EN_Z, OUTPUT);

  // Initialize microstep
  digitalWrite(MS1_X, LOW);
  digitalWrite(MS2_X, LOW);
  digitalWrite(MS1_Z, LOW);
  digitalWrite(MS2_Z, LOW);

  // Enable motors (Mark Rober disables steppers initially..?)
  digitalWrite(MOT_EN_X, LOW);
  digitalWrite(MOT_EN_Z, LOW);

  // Disable motors
  delay(100);
  disableStepperZ();
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

void sinGenerator() {
  // Generate sine path to cut
  for (int i = 0; i < num_points; ++i) {
    float y = (pathMax_y) * (float)i / (num_points - 1);
    float x = sinAmp * sinf((TWO_PI/sinPeriod)*y);
    pathArrayX[i] = x;
    pathArrayY[i] = y;
  }
}

void circleGenerator() {
  float radius = circleDiameter / 2.0;
  float angle_step = 2.0 * PI / num_points;

  for (int i = 0; i < num_points; i++) {
    float angle = i * angle_step;
    pathArrayX[i] = -radius + (radius * cosf(angle));
    pathArrayY[i] = radius * sinf(angle);
  }
}

// Loop subfunctions -----------------------------------------------------------------------------
int convTwosComp(int b){
  //Convert from 2's complement
  //if(sizeof(b) > 
  if(b & 0x80){                     // 0x80 (hex) = 10000000 (bin) = 128 (dec)
    b = -1 * ((b ^ 0xff) + 1);      // 0xff (hex) = 11111111 (bin) = 255 (dec)
  }
  return b;
}

float myDist(float x1, float y1, float x2, float y2) {
  return sqrt(pow(x1 - x2,2) + pow(y1 - y2,2));
}

float desPosIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4) {
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
  // Check if the intersection point is on the line segments
  if (t < 0 || t > 1 || u < 0 || u > 1) {
      Serial.println("Intersection point is not on the line segments.");
      return NAN;     // TO-DO: do other stuff like stopping motor
  }
  
  float x = x1 + t * (x2 - x1);
  float y = y1 + t * (y2 - y1);
//  float pointDes[2];
//  pointDes[0] = (((x1*y2 - y1*x2)*(x3 - x4)) - ((x1 - x2)*(x3*y4 - y3*x4)))/den;
//  pointDes[1] = (((x1*y2 - y1*x2)*(y3 - y4)) - ((y1 - y2)*(x3*y4 - y3*x4)))/den;

  return -myDist(xc,yc,x,y)*cosf(th);
}

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

void machineZeroX_1() {
  // First calibration run
  //myStepper.moveTo(Conv*retract);
  stepperX.setAcceleration(accel_x0);
  stepperX.move(Conv*gantryLength);
  stepperX.run();
  if (digitalRead(LIMIT_MACH_X0) == LOW) {
    // myStepper.setSpeed(0);              // stop motor
    stopStepperX();
    stepperX.setCurrentPosition(0);
    x0_count += 1;
    Serial.println("X Limit reached");
  }
}

void machineZeroX_2() {
  //myStepper.setMaxSpeed(speed_x0);
  // Retract
  stepperX.move(-Conv*retract);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  stopStepperX();
  delay(100);

  // Move in for second calibration
  stepperX.setSpeed(speed_x1);
  while (digitalRead(LIMIT_MACH_X0) == HIGH) {
    // Run until you hit the limit switch
    stepperX.runSpeed();
  }
  stopStepperX();
  stepperX.setCurrentPosition(0);
  Serial.println(stepperZ.currentPosition());

  stepperX.move(-Conv*((gantryLength/2) + xLimitOffset));
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  stepperX.setCurrentPosition(0);

  // Go back to standard settings
  stepperX.setMaxSpeed(maxVel);
  stepperX.setAcceleration(maxAccel);
  x0_count += 1;      // Stop limit switch function (should go back to 0 for main code)
}

void machineZeroZ() {
  if (z0_count == 0) { 
    // First calibration
    //stepperZ.moveTo(Conv*retract);
    stepperZ.move(Conv*zLength);
    stepperZ.run();
    if (digitalRead(LIMIT_MACH_Z0) == LOW) {
      stopStepperZ();
      stepperZ.setCurrentPosition(0);
      z0_count += 1;
      Serial.println("Limit reached");
    }
  }
  if (z0_count == 1) {
    //stepperZ.setMaxSpeed(speed_x0);
    // Retract
    stepperZ.move(-Conv*retract);
    while (stepperZ.distanceToGo() != 0) {
      stepperZ.run();
    }
    stopStepperZ();
    delay(100);
    
    // Move in for second calibration
    stepperZ.setSpeed(speed_x1);
    while (digitalRead(LIMIT_MACH_Z0) == HIGH) {
      // Run until you hit the limit switch
      stepperZ.runSpeed();
    }
    stopStepperZ();
    stepperZ.setCurrentPosition(0);
    Serial.println(stepperZ.currentPosition());
    
    z0_count += 1;      // Stop limit switch function (should go back to 0 for main code)
    disableStepperZ();  // disable so it can be zeroed to workpiece
  }
}

void workZeroZ_man() {
  // Manually set workpiece Z0
  enableStepperZ();
  stepperZ.setCurrentPosition(0);

  while (z0_count < 2) {
    stepperZ.move(Conv*zLength);
    if (z0_count == 0) { 
      // First calibration
      //stepperZ.moveTo(Conv*retract);
      stepperZ.move(Conv*zLength);
      stepperZ.run();
      if (digitalRead(LIMIT_MACH_Z0) == LOW) {
        stopStepperZ();
        z0_count += 1;
        Serial.println("Limit reached");
      }
    }
    if (z0_count == 1) {
      //stepperZ.setMaxSpeed(speed_x0);
      // Retract
      stepperZ.move(-Conv*retract);
      while (stepperZ.distanceToGo() != 0) {
        stepperZ.run();
      }
      stopStepperZ();
      delay(100);
      
      // Move in for second calibration
      stepperZ.setSpeed(speed_x1);
      while (digitalRead(LIMIT_MACH_Z0) == HIGH) {
        // Run until you hit the limit switch
        stepperZ.runSpeed();
      }
      stopStepperZ();
      maxHeight = stepperZ.currentPosition() / Conv;
      Serial.println(maxHeight);

      // Go back down to manually set height
      stepperZ.moveTo(0);
      while (abs(stepperZ.distanceToGo()) > (Conv*2)) {
        stepperZ.run();
      }
      stepperZ.setMaxSpeed(round(speed_x1/2));
      while (stepperZ.distanceToGo() !=0) {
        stepperZ.run();
      }

      stepperZ.setMaxSpeed(speed_x0);
      //disableStepperZ();
      z0_count += 1;      // Stop limit switch function (should go back to 0 for main code)
    }
  }
}

void raiseZ() {
  stepperZ.moveTo(Conv*(maxHeight - 1));
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }
  raised = 1;
}

float desiredPosition(float dX,float dY,float theta) {
  desPos = (dX - tanf(theta)*dY)*cosf(theta);       // Sanzhar equation
//desPos = myDist(estPosX,estPosY,nextX,nextY)*sinf(nextTrajC);
//        if (closest_point_index == 0 && nextX == 0 && nextY == 0) {
//          desPos = 0;
//        } else {
//          desPos = desPosIntersect(estPosX,estPosY,estYaw,lastX,lastY,nextX,nextY);
//        }
// desPos = -distance(estPosX,estPosY,nextX,nextY);
// desPos = -(nextX - estPosX);
// desPos = sinAmp * sinf((TWO_PI/sinPeriod)*estPosY);     // cheat mode
  return desPos;
}

void sensorPlotting() {
  //Serial.printf("dx:%f,dy:%f",dXmm,dYmm);
  //Serial.printf("dx:%i,dy:%i",data.dx,data.dy);
  Serial.printf("x:%f,y:%f,theta:%f",estPosX,estPosY,estYaw);
//      Serial.printf("w1:%f,w2:%f,w3:%f,w4:%f,w5:%f,w6:%f,w7:%f,w8:%f",estAngVel[0],estAngVel[1],
//        estAngVel[2],estAngVel[3],estAngVel[4],estAngVel[5],estAngVel[6],estAngVel[7]);
  //Serial.printf("x:%f,y:%f",xmm[1],ymm[1]);
  Serial.println();
}

void debugging() {
  // Put all Serial print lines here to view
//  Serial.printf("motorPos:%f,desPos:%f,index:%i,theta:%f,alpha:%f",motorPosX,desPos,
//    closest_point_index,estYaw,nextTrajC);
  Serial.printf("x:%f,y:%f,nextX:%f,nextY:%f,desPos:%i",estPosRoutX,estPosRoutY,nextX,nextY,desPos);

  Serial.println();
}
