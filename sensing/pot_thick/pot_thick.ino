#define POT_THICK       26

void setup() {
  // put your setup code here, to run once:
  pinMode(POT_THICK, INPUT);
  
  Serial.begin(115200);


}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorVal = analogRead(POT_THICK);
  float thickness = mapF(sensorVal, 0, 1024, 0, 10);
  Serial.println(thickness);
  delay(100);
}

float mapF(long x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
