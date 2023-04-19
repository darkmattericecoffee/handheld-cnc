// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 27;
const int stepPin = 33;
const int MS1 = 21;
const int MS2 = 14;
int count = 0;
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
  // Change direction once the motor reaches target position
//  if (myStepper.distanceToGo() == 0) 
//    myStepper.moveTo(-myStepper.currentPosition());
//
//  // Move the motor one step
//  myStepper.run();
  if (count == 5){
    exit(0);
  }
  myStepper.move(-1600*8*10); //One revolution at 64 microsteps is 1600*8
  while (myStepper.distanceToGo() != 0){
    myStepper.run();
  }
  myStepper.move(1600*8*10); //One revolution at 64 microsteps is 1600*8
  while (myStepper.distanceToGo() != 0){
    myStepper.run();
  }
  count = count + 1;
}
