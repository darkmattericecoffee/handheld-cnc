#include <AccelStepper.h>

#define LIMIT_SWITCH_PIN 8
#define BUTT_HANDLE 7
#define MS1  37
#define MS2  38
//#define PIN_STEP_ENABLE 13
//#define RX2 0
//#define TX2 1
//#define DIAG_PIN 12
#define dirPin 36
#define stepPin 33

int x0_count = 0;
int buttPressed = 0;

int uSteps = 64;
int Conv = 25*uSteps;             // conversion coefficient (steps/mm)
float retract = 5 * Conv;
float speed_x0 = 40.0 * Conv;             // (mm/s)
float speed_x1 = 10.0 * Conv;
float maxAccel = 100*16*2*10;

#define motorInterfaceType 1

// Motor object creation
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
 
void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  pinMode(BUTT_HANDLE, INPUT);

  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, LOW);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  myStepper.setMaxSpeed(speed_x0);
  myStepper.setAcceleration(100*16*2*10);

  myStepper.setCurrentPosition(0);

  delay(500);
}
 
void loop() {
  if (digitalRead(BUTT_HANDLE) == LOW) {
    buttPressed = 1;
  }
  
  if (buttPressed) {
    Serial.println("Limit not reached");
    
    myStepper.run();
    //limitSwitch.loop();
    if (x0_count == 0) { 
      // First calibration
      myStepper.moveTo(Conv*retract);
      myStepper.run();
      if (digitalRead(LIMIT_SWITCH_PIN) == LOW) {
        myStepper.setSpeed(0);              // stop motor
        myStepper.setCurrentPosition(0);
        x0_count += 1;
        Serial.println("Limit reached");
      }
    }
    if (x0_count == 1) {
      //myStepper.setMaxSpeed(speed_x0);
      // Retract
      myStepper.move(-retract);
      while (myStepper.distanceToGo() != 0) {
        myStepper.run();
      }
      delay(100);

      // Second calibration
      myStepper.setMaxSpeed(speed_x1);
      myStepper.run();
      if (digitalRead(LIMIT_SWITCH_PIN) == LOW) {
        myStepper.setSpeed(0);              // stop motor
        myStepper.setCurrentPosition(0);
      }
      
      Serial.println(myStepper.currentPosition());
  //    third = true;
    }
  }
//  } else {
//    myStepper.setSpeed(0);
//  }
//  if(third == true) {
//    myStepper.runSpeed();
//    if (digitalRead(LIMIT_SWITCH_PIN) == LOW) {
//      third = false;
//      fourth = true;
//    }
//  }
//  if (fourth == true) {
//    myStepper.setSpeed(0);
//  }
  
// 
//  if (digitalRead(LIMIT_SWITCH_PIN) == LOW)
//  {
//    Serial.println("LIMIT_SWITCH Activated!");
//  }else{
//    //Serial.println("Not activated.");
//  }
//
//  if (digitalRead(BUTT_HANDLE) == LOW)
//  {
//    Serial.println("BUTT_HANDLE Activated!");
//  }else{
//    //Serial.println("Not activated.");
//  }
   
  //delay(100);
}
