//#include <SpeedyStepper.h>
#include <AccelStepper.h>

// NOTE: "positive" movement for the stepper is away from the motor, so NEGATIVE in our setup

//#define LIMIT_SWITCH_PIN 2
//#define BUTT_HANDLE 7
//#define MS1  37
//#define MS2  38
//#define PIN_STEP_ENABLE 41
////#define RX2 0
////#define TX2 1
////#define DIAG_PIN 12
//#define dirPin 36
//#define stepPin 33

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

int z0_count = 2;
int x0_count = 2;
int buttPressed = 0;
int loopCount = 0;

int uSteps = 8;                  // microstep configuration
int Conv = 25*uSteps;             // conversion coefficient (steps/mm)
float retract = 5;                // (mm)
float speed_x0 = 20.0 * Conv;             // (mm/s)
float speed_x1 = 4.0 * Conv;
float accel_x0 = 200.0 * Conv;

float maxVel = 40.0 * Conv;           // max velocity motor can move at
float maxAccel = 600.0 * Conv;

// Gantry geometry
float gantryLength = 106.0;       // (mm)
float xLimitOffset = 2.54;         // distance from wall of stepper when zeroed (mm)

#define motorInterfaceType 1

// Motor object creation
AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);
//SpeedyStepper stepperZ;
 
void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_MACH_X0, INPUT);
  pinMode(LIMIT_MACH_Z0, INPUT);
  
  pinMode(BUTT_MACH_X0, INPUT);
  pinMode(BUTT_MACH_Z0, INPUT);
  pinMode(BUTT_WORK_X0Y0, INPUT);
  pinMode(BUTT_HANDLE, INPUT);

  pinMode(MOT_EN_X, OUTPUT);
  pinMode(MOT_EN_Z, OUTPUT);

//  stepperZ.connectToPins(stepPin, dirPin);
//  stepperZ.setStepsPerMillimeter(uConv);

  pinMode(MS1_X, OUTPUT);
  pinMode(MS2_X, OUTPUT);
  digitalWrite(MS1_X, LOW);
  digitalWrite(MS2_X, LOW);
  digitalWrite(MOT_EN_X, LOW);
  stepperX.setMaxSpeed(speed_x0);
  stepperX.setAcceleration(maxAccel);
  stepperX.setCurrentPosition(0);
  
  pinMode(MS1_Z, OUTPUT);
  pinMode(MS2_Z, OUTPUT);
  digitalWrite(MS1_Z, LOW);
  digitalWrite(MS2_Z, LOW);
  digitalWrite(MOT_EN_Z, LOW);
  stepperZ.setMaxSpeed(speed_x0);
  stepperZ.setAcceleration(maxAccel);
  stepperZ.setCurrentPosition(0);

  delay(500);
}
 
void loop() {
  if (digitalRead(LIMIT_MACH_Z0) == LOW) {
    Serial.println("Limit engaged!");
  }
  if (digitalRead(BUTT_HANDLE) == LOW) {
    buttPressed = 1;
  }

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
    Serial.println("z homing");
  }
  machineZeroZ();
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
    stepperZ.move(Conv*gantryLength);
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
  }
}
