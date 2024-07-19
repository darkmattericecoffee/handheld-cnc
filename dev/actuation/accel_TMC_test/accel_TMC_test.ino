#include <AccelStepper.h> //Simple & good stepper library, get it.

#include <TMCStepper.h>

#define MOT_EN_X    18
#define MOT_DIR_X   14
#define MOT_STEP_X  15
#define MS1_X       17
#define MS2_X       16
// #define DIR_PIN             6 // Direction
// #define STEP_PIN            5 // Step
#define SERIAL_PORT         Serial7 // HardwareSerial port pins 16 & 17
#define DRIVER_ADDRESS      0b00 // TMC2209 Driver address according to MS1 and MS2
// Motor properties
// Note: Constants are in units (steps/*) whereas variables are (mm/*). Kind of
// confusing, but whenedver the variables are used within an accelStepper function,
// multiply by Conv. This can be made simpler by using SpeedyStepper library.
int uSteps = 2;                       // microstep configuration
int Conv = 25*uSteps;                 // conversion factor (mm -> steps)
float stepPulseWidth = 20.0;          // min pulse width (from Mark Rober's code)
float maxCurrent_RMS = 640.0;         // motor RMS current rating (mOhm)
//float maxAccel = 800.0 * Conv;        // max acceleration (step/s^2)
float maxAccel = 40000.0;             // max acceleration (step/s^2)
#define motorInterfaceType 1

bool useDriver = 1;

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

bool shaft = false;  // ONLY NEEDED FOR CHANGING DIRECTION VIA UART, NO NEED FOR DIR PIN FOR THIS

AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
//AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

void setup() {
  //stepper.connectToPins(STEP_PIN, DIR_PIN); // INITIALIZE SpeedyStepper
    
  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209
  Serial.begin(115200);
  delay(500);
  Serial.println(F("Serial Initialized"));

  if (useDriver) {
    driver.begin();                // Initialize driver                   
    driver.toff(5);                // Enables driver in software

    driver.rms_current(640);       // Set motor RMS current
    driver.microsteps(uSteps);          // Set microsteps to 1/2

    driver.pwm_autoscale(true);    // Needed for stealthChop
    driver.en_spreadCycle(true);   // false = StealthChop / true = SpreadCycle
  } else {
    // Initialize microstep (8 microsteps)
    pinMode(MS1_X, OUTPUT);
    pinMode(MS2_X, OUTPUT);
    digitalWrite(MS1_X, LOW);
    digitalWrite(MS2_X, LOW);
  }

  // Enable motors
  pinMode(MOT_EN_X, OUTPUT);
  digitalWrite(MOT_EN_X, LOW);
  

  stepperX.setMinPulseWidth(stepPulseWidth);
  stepperX.setMaxSpeed(12500);
  stepperX.setAcceleration(maxAccel);
  stepperX.setCurrentPosition(0);
  // stepper.setCurrentPositionInSteps(0);                   // Set zero position
  // stepper.setSpeedInStepsPerSecond(400);              //Set Speed
  // stepper.setAccelerationInStepsPerSecondPerSecond(400);   //Set acceleration, smaller value for super smooth direction changing
  delay(800);

}

void loop() {
  // uint16_t msread=driver.microsteps();
  // Serial.print(F("Read microsteps via UART to test UART receive : "));    Serial.println(msread); 
    
  // Serial.println(F("Move 6400 steps forward at 600ma"));
  // driver.rms_current(600); 
  // stepper.moveToPositionInSteps(6400);

  // Serial.println(F("Wait 3sec and turn current low so you can turn the motor shaft"));
  // driver.rms_current(10); 
  // delay(3000);

  // Serial.println(F("Move back to 0 position at 300ma"));
  // driver.rms_current(300); 
  // stepper.moveToPositionInSteps(0);

  //
  // Here will will try rotating at faster and faster speeds.  You can
  // see how the motor's torque is reduced as the speed increases by
  // pinching the shaft with your fingers.  Note: When the motor can't
  // go at the commanded speed, it simply stops.
  //
  // I have conducted all of these tests with 1x microstepping.
  //

  // //
  // // 100 steps/second
  // //
  // Serial.println("Testing 100 steps/second.");
  // stepperX.setMaxSpeed(100);
  // stepperX.setAcceleration(100);
  // stepperX.move(200);
  // while (stepperX.distanceToGo() != 0) {
  //   stepperX.run();
  // }
  // delay(800);

  // //
  // // 200 steps/second
  // //
  // Serial.println("Testing 200 steps/second.");
  //   stepperX.setMaxSpeed(200);
  // stepperX.setAcceleration(200);
  // stepperX.move(400);
  // while (stepperX.distanceToGo() != 0) {
  //   stepperX.run();
  // }
  // delay(800);

  //
  // 400 steps/second
  //
  Serial.println("Testing 400 steps/second.");
  stepperX.setMaxSpeed(400);
  // stepperX.setAcceleration(400);
  stepperX.move(800);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(800);

  //
  // 800 steps/second
  // Note: My 12V NEAMA 17 motor with a 20V power supply stops here
  //
  Serial.println("Testing 800 steps/second.");
  stepperX.setMaxSpeed(800);
  // stepperX.setAcceleration(800);
  stepperX.move(1600);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(800);

  //
  // 1600 steps/second
  //
  Serial.println("Testing 1600 steps/second.");
  stepperX.setMaxSpeed(1600);
  // stepperX.setAcceleration(1600);
  stepperX.move(3200);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(800);

  //
  // 3200 steps/second
  //
  Serial.println("Testing 3200 steps/second.");
  stepperX.setMaxSpeed(3200);
  // stepperX.setAcceleration(3200);
  stepperX.move(6400);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(800);

  //
  // 6400 steps/second
  // Note: My 3.2V NEAMA 17 motor with a 20V power supply stops here
  //
  Serial.println("Testing 6400 steps/second.");
  stepperX.setMaxSpeed(6400);
  // stepperX.setAcceleration(6400);
  stepperX.move(12800);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(800);

  //
  //
  //
  Serial.println("Testing 8000 steps/second.");
  stepperX.setMaxSpeed(8000);
  // stepperX.setAcceleration(8000);
  stepperX.move(16000);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(800);

  //
  // 12500 steps/second, the fastest this library can generate
  // Note: My 3.8V NEAMA 11 motor with a 20V power supply stops here
  //
  Serial.println("Testing 12500 steps/second.");
  stepperX.setMaxSpeed(12500);
  // stepperX.setAcceleration(6400);
  stepperX.move(38400);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(800);

  Serial.println("");  
  delay(3000);

}