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
int Conv = 250*8;
int myArray[] = {2,-2,2,-20};
int vel[] = {1,-2,1,-5};
int size = 4;
bool began = false;
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
  digitalWrite(MS2, LOW);
  myStepper.setMaxSpeed(12800*20);
  myStepper.setAcceleration(100*16*2*10);
 // myStepper.setSpeed(200);
  //myStepper.moveTo(200);
  myStepper.setCurrentPosition(0);
 
  

}

void loop() {
  if (began == false) {
    began = true;
    myStepper.moveTo(Conv*myArray[count]);
    myStepper.setMaxSpeed(vel[count]*Conv);
  } else {
    if (myStepper.distanceToGo() != 0) {
      //delay(100);
      myStepper.run();
    } else {
      began = false;
      count = count + 1;
      delay(1000);
      if(count == size) {
        count = 0;
      }
    }
  }
}
