float Cx[4] = {0.01003310926f,0.009966115208f,0.01008674602f,0.009975062344f};
int b = 4;
float a = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  a = b*Cx[3];
  Serial.println(a);
}

void loop() {
  // put your main code here, to run repeatedly:

}
