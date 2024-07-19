// Testing speed of SpeedyStepper over AccelStepper

#include <SpeedyStepper.h> //Simple & good stepper library, get it.

#include <TMCStepper.h>

#define MOT_EN_X    18
#define MOT_DIR_X   14
#define MOT_STEP_X  15
// #define DIR_PIN             6 // Direction
// #define STEP_PIN            5 // Step
#define SERIAL_PORT         Serial7 // HardwareSerial port pins 16 & 17
#define DRIVER_ADDRESS      0b00 // TMC2209 Driver address according to MS1 and MS2
int uSteps = 2;                       // microstep configuration
int Conv = 25*uSteps;                 // conversion factor (mm -> steps)
float stepPulseWidth = 20.0;          // min pulse width (from Mark Rober's code)
float maxCurrent_RMS = 640.0;         // motor RMS current rating (mOhm)
//float maxAccel = 800.0 * Conv;        // max acceleration (step/s^2)
float maxAccel = 40000.0;             // max acceleration (step/s^2)
// #define motorInterfaceType 1

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

bool shaft = false;  // ONLY NEEDED FOR CHANGING DIRECTION VIA UART, NO NEED FOR DIR PIN FOR THIS


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

SpeedyStepper stepper;

void setup() {
  stepper.connectToPins(MOT_STEP_X, MOT_DIR_X); // INITIALIZE SpeedyStepper
    
  SERIAL_PORT.begin(115200);      // INITIALIZE UART TMC2209
  Serial.begin(115200);
  delay(500);
  Serial.println(F("Serial Initialized"));

  driver.begin();                // Initialize driver                   
  driver.toff(5);                // Enables driver in software

  driver.rms_current(640);       // Set motor RMS current
  driver.microsteps(2);          // Set microsteps to 1/2

  driver.pwm_autoscale(true);    // Needed for stealthChop
  driver.en_spreadCycle(true);   // false = StealthChop / true = SpreadCycle

  // Enable motors
  pinMode(MOT_EN_X, OUTPUT);
  digitalWrite(MOT_EN_X, LOW);

  stepper.setCurrentPositionInSteps(0);                   // Set zero position
  stepper.setSpeedInStepsPerSecond(400);              //Set Speed
  stepper.setAccelerationInStepsPerSecondPerSecond(maxAccel);   //Set acceleration, smaller value for super smooth direction changing

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
  // stepper.setSpeedInStepsPerSecond(100);
  // stepper.setAccelerationInStepsPerSecondPerSecond(100);
  // stepper.moveRelativeInSteps(200);
  // delay(800);

  // //
  // // 200 steps/second
  // //
  // Serial.println("Testing 200 steps/second.");
  // stepper.setSpeedInStepsPerSecond(200);
  // stepper.setAccelerationInStepsPerSecondPerSecond(200);
  // stepper.moveRelativeInSteps(400);
  // delay(800);

  //
  // 400 steps/second
  //
  Serial.println("Testing 400 steps/second.");
  stepper.setSpeedInStepsPerSecond(400);
  // stepper.setAccelerationInStepsPerSecondPerSecond(400);
  stepper.moveRelativeInSteps(800);
  delay(800);

  //
  // 800 steps/second
  // Note: My 12V NEAMA 17 motor with a 20V power supply stops here
  //
  Serial.println("Testing 800 steps/second.");
  stepper.setSpeedInStepsPerSecond(800);
  // stepper.setAccelerationInStepsPerSecondPerSecond(800);
  stepper.moveRelativeInSteps(1600);
  delay(800);

  //
  // 1600 steps/second
  //
  Serial.println("Testing 1600 steps/second.");
  stepper.setSpeedInStepsPerSecond(1600);
  // stepper.setAccelerationInStepsPerSecondPerSecond(1600); 
  stepper.moveRelativeInSteps(3200);
  delay(800);

  //
  // 3200 steps/second
  //
  Serial.println("Testing 3200 steps/second.");
  stepper.setSpeedInStepsPerSecond(3200);
  // stepper.setAccelerationInStepsPerSecondPerSecond(3200); 
  stepper.moveRelativeInSteps(6400);
  delay(800);

  //
  // 6400 steps/second
  // Note: My 3.2V NEAMA 17 motor with a 20V power supply stops here
  //
  Serial.println("Testing 6400 steps/second.");
  stepper.setSpeedInStepsPerSecond(6400);
  // stepper.setAccelerationInStepsPerSecondPerSecond(6400); 
  stepper.moveRelativeInSteps(12800);
  delay(800);

  //
  // 12500 steps/second, the fastest this library can generate
  // Note: My 3.8V NEAMA 11 motor with a 20V power supply stops here
  //
  Serial.println("Testing 12500 steps/second.");
  stepper.setSpeedInStepsPerSecond(12500);
  // stepper.setAccelerationInStepsPerSecondPerSecond(6400);
  stepper.moveRelativeInSteps(38400);
  delay(800);

  Serial.println("");  
  delay(3000);

}