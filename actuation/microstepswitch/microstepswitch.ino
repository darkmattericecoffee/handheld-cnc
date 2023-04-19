// Define pin connections & motor's steps per revolution
#include <ezButton.h>

ezButton limitSwitch(15); 
#define PIN_STEP_ENABLE 13
const int dirPin = 27;
const int stepPin = 33;
const int stepsPerRevolution = 200;
const int MS1 = 21;
const int MS2 = 14;

void iterate(int arr[], int size){
  delay(2000);
  for (int i = 0; i<size; i++){
    Serial.println(String(arr[i]));
    if (arr[i]<0) {
      digitalWrite(dirPin, HIGH);
    }
    else {
      digitalWrite(dirPin, LOW);
    }
 
    for(int x = 0; x < (12800/.8)*abs(arr[i]); x++)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(20);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(20);
    }
    delay(2000);
  }
}
void setup()
{
  // Declare pins as Outputs
  Serial.begin(115200);
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, LOW);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  int myArray[] = {5,-5,5,-5};
  
digitalWrite(dirPin,HIGH);
  int arraySize = 4;
 iterate(myArray,arraySize);

}
void loop()
{
   digitalWrite(stepPin, HIGH);
      delayMicroseconds(20);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(20);
  
}
