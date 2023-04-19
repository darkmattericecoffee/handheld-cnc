#define LIMIT_SWITCH_PIN 8
#define BUTT_HANDLE 7
 
void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  pinMode(BUTT_HANDLE, INPUT);
}
 
void loop() {
 
  if (digitalRead(LIMIT_SWITCH_PIN) == LOW)
  {
    Serial.println("LIMIT_SWITCH Activated!");
  }else{
    //Serial.println("Not activated.");
  }

  if (digitalRead(BUTT_HANDLE) == LOW)
  {
    Serial.println("BUTT_HANDLE Activated!");
  }else{
    //Serial.println("Not activated.");
  }
   
  //delay(100);
}
