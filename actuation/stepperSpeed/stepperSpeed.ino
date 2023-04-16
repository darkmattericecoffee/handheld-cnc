// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
//const int PIN_STEP_ENABLE 13;
//const int dirPin = 27;
//const int stepPin = 33;
#define MS1  21
#define MS2  14
#define PIN_STEP_ENABLE 13
//#define RX2 0
//#define TX2 1
//#define DIAG_PIN 12
#define dirPin 27
#define stepPin 33
#define SERIAL_PORT        Serial1    // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS     0b00       // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE            0.11f 
int count = 0;
int Conv = 250*64;
int myArray[] = {5,-2,2,-5};
int vel[] = {2,1,3,4};
int size = 4;
// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, LOW);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  myStepper.setMaxSpeed(12800*2);
  myStepper.setAcceleration(100*16*3*10);
 // myStepper.setSpeed(200);
  //myStepper.moveTo(200);
  myStepper.setCurrentPosition(0);
 
  

}

void loop() {
  myStepper.setSpeed(vel[count]*Conv);
  myStepper.moveTo(Conv*myArray[count]);
  count = count + 1;
  if(count == size - 1) {
    count = 0;
  }
}
