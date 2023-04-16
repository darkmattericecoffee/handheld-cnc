// Define pin connections & motor's steps per revolution
#define PIN_STEP_ENABLE 13
const int dirPin = 27;
const int stepPin = 33;
const int stepsPerRevolution = 200;
const int MS1 = 21;
const int MS2 = 14;

//void iterate(int arr[], int size){
//  delay(2000);
//  for (int i = 0; i<size; i++){
//    Serial.println(String(arr[i]));
//    if (arr[i]<0) {
//      digitalWrite(dirPin, HIGH);
//    }
//    else {
//      digitalWrite(dirPin, LOW);
//    }
//    double turns = abs(arr[i]/0.8);
//    for(int x = 0; x < turns*stepsPerRevolution; x++)
//    {
//      digitalWrite(stepPin, HIGH);
//      delayMicroseconds(2000);
//      digitalWrite(stepPin, LOW);
//      delayMicroseconds(2000);
//    }
//    delay(2000);
//  }
//}
void setup()
{
  // Declare pins as Outputs
  Serial.begin(115200);
  //pinMode(MS1, OUTPUT);
  //digitalWrite(MS1, HIGH);
  //pinMode(MS2, OUTPUT);
  //digitalWrite(MS2, HIGH);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  //int myArray[] = {5,-5,5,-5};
  
digitalWrite(dirPin,HIGH);
  int arraySize = 4;
 // iterate(myArray,arraySize);

}
void loop()
{
   digitalWrite(stepPin, HIGH);
      delayMicroseconds(2000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(2000);
  
}
