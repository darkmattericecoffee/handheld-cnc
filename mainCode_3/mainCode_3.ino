// Libraries to include
#include <PMW3360.h>
#include <AccelStepper.h>
/*
Sensor configuration (USING 3 SENSORS NOW!!):
0 --- 1              ^ y
|  x  |   (old x <-) |-> x
2 --- (3 gone)
    NOTE: old config didn't obey the right hand rule. We are now flipping x.
*/

// Pin definitions -------------------------------------------------------------------------------
// Sensor pins
#define SS0   39   // Chip select pin. Connect this to SS on the module.
#define SS1   10
#define SS2   40
//#define SS3   32
#define LIMIT_MACH_X0   8
#define BUTT_MACH_X0    4
#define BUTT_MACH_Z0    3
#define BUTT_WORK_X0Y0   6
#define BUTT_WORK_Z0   5
#define BUTT_HANDLE     7
#define POT_THICK   31

// Motor Pins
#define MS1  37
#define MS2  38
#define STEP_ENABLE 41
//#define RX2 0
//#define TX2 1
//#define DIAG_PIN 12
int dirPin = 36;
int stepPin = 33;

// Constants ------------------------------------------------------------------------
// Sensor properties
const int ns = 3;                   // number of sensors
const int CPI = 2500;               // This value changes calibration coefficients
const float Cx[3] = {0.00997506234f,0.01003310926f,0.00996611521f};
const float Cy[3] = {0.01011531459f,0.01026588646f,0.01019056354f};
const float lx = 120.0f;                // x length of rectangular sensor configuration
const float ly = 140.0f;                // y length of rectangular sensor configuration
const float xOff[3] = {-lx/2,lx/2,-lx/2};
const float yOff[3] = {ly/2,ly/2,-ly/2};

// Motor properties
int count = 0;
int uSteps = 64;
int Conv = 25*uSteps;
//int pos[] = {20,-20,20,-20};
//int vel[] = {10,-20,10,-40};
float maxVel = 40.0 * Conv;           // max velocity motor can move at
float maxAccel = 200.0 * Conv;
float retract = 5;                // (mm)
float speed_x0 = 20.0 * Conv;             // x zeroing speed (mm/s)
float speed_x1 = 4.0 * Conv;              // x secondary zeroing speed
bool began = false;
#define motorInterfaceType 1

// Gantry geometry
float gantryLength = 106.0;       // (mm)
float xLimitOffset = 2.54;         // distance from wall of stepper when zeroed (mm)

// Variables ------------------------------------------------------------------------
int plotting = 1;

// Button states
int readyOrNot = 0;

// Zeroing variables
int x0_count = 2;           // x zeroing count variable (start as "false")

int firstPoint = 1;
long unsigned timeLastPoll = 0;
long unsigned dt = 500;       // microseconds (freq = 1,000,000/timestepPoll [Hz])

// Measured quantities
int dX[3] = {0,0,0};
int dY[3] = {0,0,0};
float dXmm[3] = {0.0f,0.0f,0.0f};
float dYmm[3] = {0.0f,0.0f,0.0f};
float vXmm[3] = {0.0f,0.0f,0.0f};        // velocity (mm/s)(dXmm/dt)
float vYmm[3] = {0.0f,0.0f,0.0f};

// Estimated quantities
float xmm[3] = {0.0f,0.0f,0.0f};         // *not significant*
float ymm[3] = {0.0f,0.0f,0.0f};         // *not significant*
float estVelX[3] = {0.0f,0.0f,0.0f};
float estVelY[3] = {0.0f,0.0f,0.0f};
float estVelX1 = 0.0f;
float estVelY1 = 0.0f;
float estPosX = 0.0f;                         // router center position - x
float estPosY = 0.0f;                         // router center position - y
float estAngVel[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float estAngVel1 = 0.0f;
float estYaw = 0.0f;                          // Angle estimation

// Control Constants ----------------------------------------------------------------
const int len = 1000;       // length of path array

// Control Variables ----------------------------------------------------------------
// Motor control variables
float distDes = 0.0f;

// Object Initialization ------------------------------------------------------------
// Sensor object creation
PMW3360 sensor0;
PMW3360 sensor1;
PMW3360 sensor2;
//PMW3360 sensor3;

// Motor object creation
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin(9600);  
  while(!Serial);

  // Button initialization
  pinMode(LIMIT_MACH_X0, INPUT);
  pinMode(BUTT_MACH_X0, INPUT);
  pinMode(BUTT_WORK_X0Y0, INPUT);
  pinMode(BUTT_HANDLE, INPUT);
  pinMode(STEP_ENABLE, OUTPUT);
  //pinMode(LIMIT_WOR

  sensorSetup();

  motorSetup();
  
  delay(500);
}

void loop() {
  //Serial.println(digitalRead(LIMIT_MACH_X0));
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

  // Workpiece zeroing
  //Serial.println(digitalRead(BUTT_WORK_X0Y0));
  if (digitalRead(BUTT_WORK_X0Y0) == LOW) {
    readyOrNot = 1;
  }

  // Sensor polling
  if (readyOrNot) {
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
        dXmm[0] = -convTwosComp(data0.dx)*Cx[0] / dt;       // maybe divide by dt (would be slower, but more robust?)
        dXmm[1] = -convTwosComp(data1.dx)*Cx[1] / dt;
        dXmm[2] = -convTwosComp(data2.dx)*Cx[2] / dt;
        //dXmm[3] = -convTwosComp(data3.dx)*Cx[3] / dt;
        dYmm[0] = convTwosComp(data0.dy)*Cx[0] / dt;
        dYmm[1] = convTwosComp(data1.dy)*Cx[1] / dt;
        dYmm[2] = convTwosComp(data2.dy)*Cx[2] / dt;
        //dYmm[3] = convTwosComp(data3.dy)*Cx[3] / dt;
  
  //      dXmm = dX*Cx[3];
  //      dYmm = dY*Cy[3];
  
        // Sensor position estimation
  //      if(firstPoint) {
  //        // make sure first point (x,y) is (0,0) (there could be a better way to do this)
  //        firstPoint = 0;
  //      }else{
  //        for (int i = 0; i<4; i++) {
  //          xmm[i] = xmm[i] + dXmm[i];
  //          ymm[i] = ymm[i] + dYmm[i];
  //        }
  //      }
  
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
        estVelX1 = sumVelX / ns;
        estVelY1 = sumVelY / ns;
        estPosX = estPosX + estVelX1*dt;
        estPosY = estPosY + estVelY1*dt;
        
        float absVel = sqrt(pow(dXmm[0],2) + pow(dYmm[0],2));
        //Serial.printf("Absolute velocity = %f",absVel);
        //Serial.println();
  
        // Sensor plotting
        if (plotting) {
          sensorPlotting();
        }
      }
    }
  
    // Motor Control ---------------------------------------------------------------------------------
    distDes = estPosX*cos(estYaw);
    
    if (digitalRead(BUTT_HANDLE) == LOW) {
      Serial.println("User in control!");
      // User is in control of device, run control code
      if (began == false) {
        began = true;
        myStepper.moveTo(Conv*distDes);
        //myStepper.setMaxSpeed(maxVel);
      } else {
        if (myStepper.distanceToGo() != 0) {
          //delay(100);
          myStepper.run();
        } else {
          began = false;
        }
      }
    } else {
      // User is not in control of device, cancel everything
      myStepper.setSpeed(0);
      //readyOrNot = 0;
      Serial.println("User not in control!");
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

  // Initialize sensor locations relative to body
  Serial.print("Initial locations of sensors: ");
  for (int i = 0; i<ns; i++) {
    xmm[i] = xmm[i] + xOff[i];
    ymm[i] = ymm[i] + yOff[i];
    Serial.printf("%i(%.2f,%.2f),",i+1,xmm[i],ymm[i]);
  }
  Serial.println();
}

void motorSetup() {
  // Setup motors
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, LOW);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  digitalWrite(STEP_ENABLE, LOW);
  myStepper.setMaxSpeed(speed_x0);
  myStepper.setAcceleration(maxAccel);
  myStepper.setCurrentPosition(0);
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

void machineZeroX_1() {
  // First calibration run
  //myStepper.moveTo(Conv*retract);
  myStepper.move(-Conv*gantryLength);
  myStepper.run();
  if (digitalRead(LIMIT_MACH_X0) == LOW) {
    myStepper.setSpeed(0);              // stop motor
    myStepper.setCurrentPosition(0);
    x0_count += 1;
    Serial.println("Limit reached");
  }
}

void machineZeroX_2() {
  //myStepper.setMaxSpeed(speed_x0);
  // Retract
  myStepper.move(Conv*retract);
  while (myStepper.distanceToGo() != 0) {
    myStepper.run();
  }
  myStepper.setSpeed(0);
  myStepper.runSpeed();
  delay(100);

  // Move in for second calibration
  myStepper.setSpeed(-speed_x1);
  while (digitalRead(LIMIT_MACH_X0) == HIGH) {
    // Run until you hit the limit switch
    myStepper.runSpeed();
  }
  myStepper.setSpeed(0);              // stop motor
  myStepper.runSpeed();
  myStepper.setCurrentPosition(0);
  Serial.println(myStepper.currentPosition());

  myStepper.move(Conv*((gantryLength/2) - xLimitOffset));
  while (myStepper.distanceToGo() != 0) {
    myStepper.run();
  }
  myStepper.setCurrentPosition(0);

  // Go back to standard settings
  myStepper.setMaxSpeed(maxVel);
  x0_count += 1;      // Stop limit switch function (should go back to 0 for main code)
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
