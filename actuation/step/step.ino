// Define pin connections & motor's steps per revolution
#define PIN_STEP_ENABLE 13
const int DIR_PIN = 27;
const int STEP_PIN= 33;

const int stepsPerRevolution = 200;
const int MS1 = 21;
const int MS2 = 14;

//void iterate(int arr[], int size){
//  delay(2000);
//  for (int i = 0; i<size; i++){
//    Serial.println(String(arr[i]));
//    if (arr[i]<0) {
//      digitalWrite(DIR_PIN, HIGH);
//    }
//    else {
//      digitalWrite(DIR_PIN, LOW);
//    }
//    double turns = abs(arr[i]/0.8);
//    for(int x = 0; x < turns*stepsPerRevolution; x++)
//    {
//      digitalWrite(stepPin, HIGH);
//      delayMicroseconds(1000);
//      digitalWrite(stepPin, LOW);
//      delayMicroseconds(1000);
//    }
//    delay(2000);
//  }
//}
void setup()
{
  // Declare pins as Outputs
  Serial.begin(115200);
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, LOW);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  int myArray[] = {2,-2,2,-2};
  int arraySize = 4;
  //iterate(myArray,arraySize);

}
bool shaft = true;

void loop() {
  // Run 5000 steps and switch direction in software
  for (uint16_t i = 1600*8; i>0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(300/4);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(300/4);
  }
 delay(200);
  if (shaft==true) {
    shaft = false;
    digitalWrite(DIR_PIN,HIGH);
  } else {
    shaft = true;
    digitalWrite(DIR_PIN,LOW);
  }
  
  //driver.shaft(shaft);
}
  
