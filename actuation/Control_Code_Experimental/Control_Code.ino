#include <PMW3360.h>
#include <AccelStepper.h>
/*
Sensor configuration:
0 --- 1 ^ y
| x | (old x <-) |-> x
2 --- 3
 NOTE: old config didn't obey the right hand rule. We are now flipping x.
*/

// Pin definitions
#define SS0 41 // Chip select pin. Connect this to SS on the module.
#define SS1 39
#define SS2 10
#define SS3 40
#define STEP_PIN 20
#define DIR_PIN 19
#define MS1 23
#define MS2 22




// Sensor properties
int CPI = 2500; // This value changes calibration coefficients
float Cx[4] = {0.00997506234f,0.01003310926f,0.00996611521f,0.01008674602f};
float Cy[4] = {0.01011531459f,0.01026588646f,0.01019056354f,0.01016570093f};
float l = 100.0f; // length of square sensor configuration

// Constants ------------------------------------------------------------------------
const float xOff[4] = {-l/2,l/2,-l/2,l/2};
const float yOff[4] = {l/2,l/2,-l/2,-l/2};

// Variables ------------------------------------------------------------------------
int plotting = 1;

int firstPoint = 1;
long unsigned timeLastPoll = 0;
long unsigned dt = 500; // microseconds (freq = 1,000,000/timestepPoll [Hz])

// Measured quantities
int dX[4] = {0,0,0,0};
int dY[4] = {0,0,0,0};
float dXmm[4] = {0.0f,0.0f,0.0f,0.0f};
float dYmm[4] = {0.0f,0.0f,0.0f,0.0f};
float vXmm[4] = {0.0f,0.0f,0.0f,0.0f}; // velocity (mm/s)(dXmm/dt)
float vYmm[4] = {0.0f,0.0f,0.0f,0.0f};

// Estimated quantities
float xmm[4] = {0.0f,0.0f,0.0f,0.0f}; // *not significant*
float ymm[4] = {0.0f,0.0f,0.0f,0.0f}; // *not significant*
float estVelX[4] = {0.0f,0.0f,0.0f,0.0f};
float estVelY[4] = {0.0f,0.0f,0.0f,0.0f};
float estVelX1 = 0.0f;
float estVelY1 = 0.0f;
float estPosX = 0.0f; // router center position - x
float estPosY = 0.0f; // router center position - y
float estAngVel[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float estAngVel1 = 0.0f;
float estYaw = 0.0f; // Angle estimation



// Control Constants ----------------------------------------------------------------
const int len = 1000;

// Control Variables ----------------------------------------------------------------
float posgant = 0;
float velgant;
float g[2][len];
float R[2][2];  // 2 by 2 rotational matrix 
int c = 0;
float posRoutF[2]; //position of the router in frame F, vector of x,y
float posrout[2]; //position of the router in frame T, vector of x,y
float velrout[2]; //velocity of the router in x direction and y direction
float x[len];
float y[len];
float velpe[2];
float velpa[2];
float temp[2];
float orient[2];
float minDist;
float gf[2];
float invR[2][2];
float gf_frame[2];
float dot;
float normVelpe;
float normVelpa;

// Object Initialization ------------------------------------------------------------
PMW3360 sensor0;
PMW3360 sensor1;
PMW3360 sensor2;
PMW3360 sensor3;
AccelStepper myStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
 Serial.begin(9600); 
 // put your setup code here, to run once:

 float start = 0;
 float end = 50;
 float step = (end - start) / (len - 1);

 for (int i = 0; i < len; i++) {
    x[i] = start + i * step;
    y[i] = sin(0.2 * x[i]) + 1.5;
    g[0][i] = x[i];
    g[1][i] = y[i];
 }

 pinMode(MS1, OUTPUT);
 digitalWrite(MS1, LOW);
 pinMode(MS2, OUTPUT);
 digitalWrite(MS2, HIGH);
 myStepper.setMaxSpeed(12800*2); //these values have to be adjusted
 myStepper.setAcceleration(100*16*3*10); // these values have to be adjusted
 myStepper.setCurrentPosition(0);
 
 while(!Serial);
 
 // Sensor initialization
 if(sensor0.begin(SS0, CPI)) { // 10 is the pin connected to SS of the module.
 Serial.print("Sensor0 initialization successed, with CPI = ");
 Serial.println(sensor0.getCPI());
 }else{
 Serial.println("Sensor0 initialization failed");
 }
 if(sensor1.begin(SS1, CPI)) { // 10 is the pin connected to SS of the module.
 Serial.print("Sensor1 initialization successed, with CPI = ");
 Serial.println(sensor1.getCPI());
 }else{
 Serial.println("Sensor1 initialization failed");
 }
 if(sensor2.begin(SS2, CPI)) { // 10 is the pin connected to SS of the module.
 Serial.print("Sensor2 initialization successed, with CPI = ");
 Serial.println(sensor2.getCPI());
 }else{
 Serial.println("Sensor2 initialization failed");
 }
 if(sensor3.begin(SS3, CPI)) { // 10 is the pin connected to SS of the module.
 Serial.print("Sensor3 initialization successed, with CPI = ");
 Serial.println(sensor3.getCPI());
 }else{
 Serial.println("Sensor3 initialization failed");
 }

 // Initialize sensor locations relative to body
 Serial.print("Initial locations of sensors: ");
 for (int i = 0; i<4; i++) {
 xmm[i] = xmm[i] + xOff[i];
 ymm[i] = ymm[i] + yOff[i];
 Serial.printf("%i(%.2f,%.2f),",i+1,xmm[i],ymm[i]);
 }
 Serial.println();
 
 delay(500);
}

void loop() {
 if(micros() - timeLastPoll >= dt) {
 //Serial.println(micros() - timeLastPoll);
 timeLastPoll = micros();

 // Sensing ---------------------------------------------------------------------
 // Collect sensor data
 PMW3360_DATA data0 = sensor0.readBurst_simple();
 PMW3360_DATA data1 = sensor1.readBurst_simple();
 PMW3360_DATA data2 = sensor2.readBurst_simple();
 PMW3360_DATA data3 = sensor3.readBurst_simple();

 int yup = 1;
 if(yup) {
 //if(data1.isOnSurface && data1.isMotion) { // If movement...
 // **Only checking sensor 1 rn (TODO: check all of them and account for misreads)**

 // Sensor velocity sensing
 dXmm[0] = -convTwosComp(data0.dx)*Cx[0] / dt; // maybe divide by dt (would be slower, but more robust?)
 dXmm[1] = -convTwosComp(data1.dx)*Cx[1] / dt;
 dXmm[2] = -convTwosComp(data2.dx)*Cx[2] / dt;
 dXmm[3] = -convTwosComp(data3.dx)*Cx[3] / dt;
 dYmm[0] = convTwosComp(data0.dy)*Cx[0] / dt;
 dYmm[1] = convTwosComp(data1.dy)*Cx[1] / dt;
 dYmm[2] = convTwosComp(data2.dy)*Cx[2] / dt;
 dYmm[3] = convTwosComp(data3.dy)*Cx[3] / dt;



// dXmm = dX*Cx[3];
// dYmm = dY*Cy[3];

 // Sensor position estimation
// if(firstPoint) {
// // make sure first point (x,y) is (0,0) (there could be a better way to do this)
// firstPoint = 0;
// }else{
// for (int i = 0; i<4; i++) {
// xmm[i] = xmm[i] + dXmm[i];
// ymm[i] = ymm[i] + dYmm[i];
// }
// }

 // Body angle estimation
 estAngVel[0] = (dXmm[2] - dXmm[0])/l;
 estAngVel[1] = (dXmm[3] - dXmm[0])/l;
 estAngVel[2] = (dXmm[2] - dXmm[1])/l;
 estAngVel[3] = (dXmm[3] - dXmm[1])/l;
 estAngVel[4] = (dYmm[1] - dYmm[0])/l;
 estAngVel[5] = (dYmm[3] - dYmm[0])/l;
 estAngVel[6] = (dYmm[1] - dYmm[2])/l;
 estAngVel[7] = (dYmm[3] - dYmm[2])/l;
 float sumAngVel = 0.0f;
 for (int i = 0; i<8; i++) {
 // Simple average of angular velocities
 sumAngVel = sumAngVel + estAngVel[i];
 //Serial.printf("w%i:%f,",i,estAngVel[i]);
 }
 estAngVel1 = sumAngVel / 8.0f;
 estYaw = estYaw + estAngVel1*dt;
 //Serial.printf("w:%f",estAngVel1);

 // Body position estimation
 estVelX[0] = dXmm[0]*cosf(estYaw)-dYmm[0]*sinf(estYaw) + 0.5*estAngVel1*l*(-sinf(estYaw)+cosf(estYaw));
 estVelX[1] = dXmm[1]*cosf(estYaw)-dYmm[1]*sinf(estYaw) + 0.5*estAngVel1*l*(sinf(estYaw)+cosf(estYaw));
 estVelX[2] = dXmm[2]*cosf(estYaw)-dYmm[2]*sinf(estYaw) + 0.5*estAngVel1*l*(-sinf(estYaw)-cosf(estYaw));
 estVelX[3] = dXmm[3]*cosf(estYaw)-dYmm[3]*sinf(estYaw) + 0.5*estAngVel1*l*(sinf(estYaw)-cosf(estYaw));
 estVelY[0] = dXmm[0]*sinf(estYaw)+dYmm[0]*cosf(estYaw) + 0.5*estAngVel1*l*(cosf(estYaw)+sinf(estYaw));
 estVelY[1] = dXmm[1]*sinf(estYaw)+dYmm[1]*cosf(estYaw) + 0.5*estAngVel1*l*(-cosf(estYaw)+sinf(estYaw));
 estVelY[2] = dXmm[2]*sinf(estYaw)+dYmm[2]*cosf(estYaw) + 0.5*estAngVel1*l*(cosf(estYaw)-sinf(estYaw));
 estVelY[3] = dXmm[3]*sinf(estYaw)+dYmm[3]*cosf(estYaw) + 0.5*estAngVel1*l*(-cosf(estYaw)-sinf(estYaw));
 float sumVelX = 0.0f;
 float sumVelY = 0.0f;
 for (int i = 0; i<4; i++) {
 // Simple average of angular velocities
 sumVelX = sumVelX + estVelX[i];
 sumVelY = sumVelY + estVelY[i];
 }
 
 estVelX1 = sumVelX / 4.0f;
 estVelY1 = sumVelY / 4.0f;
 estPosX = estPosX + estVelX1*dt;
 estPosY = estPosY + estVelY1*dt;

 // Controld Code
 R[0][0] = cos(estYaw);
 R[0][1] = sin(estYaw);
 R[1][0] = -sin(estYaw);
 R[1][1] = cos(estYaw);

 orient[0] = R[0][0] + R[0][1];
 orient[1] = R[1][0] + R[1][1];
 posRoutF[0] = 0;
 posRoutF[1] = 1;


 for (int i = 0; i < 2; i++) {
  for (int j = 0; j < 2; j++) {
   temp[i] += R[i][j] * posRoutF[j];
  }
 }

 posrout[0] = temp[0] + estPosX;
 posrout[1] = temp[1] + estPosY;
 velrout[0] = estVelX1;
 velrout[1] = estVelY1;
 dot = velrout[0]*orient[0] + velrout[1]*orient[1];
 velpe[0] = orient[0]*dot;
 velpe[1] = orient[1]*dot;
 velpa[0] = velrout[0] - velpe[0];
 velpa[1] = velrout[1] - velpe[1];
 minDist = abs(posrout[0] - x[0]);
 int index = 0;
 for (int i = 1; i < len; i++) {
  float dist = abs(posrout[0] - x[i]);
  if (dist < minDist && y[i]>=posrout[1]) {
    minDist = dist;
    index = i;
  }
 }
 int target = index + 1;
 for (int i = 0; i < 2; i++) {
  gf[i] = g[i][target] - posrout[i];
 }

 invR[0][0] = R[1][1];
 invR[0][1] = -R[0][1];
 invR[1][0] = -R[1][0];
 invR[1][1] = R[0][0];
 
 for (int i = 0; i < 2; i++) {
  for (int j = 0; j < 2; j++) {
   gf_frame[i] += invR[i][j] * gf[j];
  }
 }
 normVelpe = velpe[0]*velpe[0] + velpe[1]*velpe[1];
 normVelpe = sqrt(normVelpe);
 normVelpa = velpa[0]*velpa[0] + velpa[1]*velpa[1];
 normVelpa = sqrt(normVelpa);
 velgant = normVelpe / (gf[0] / gf[1]) - normVelpa;
 
 myStepper.setSpeed(velgant);
 unsigned long startTime = millis();
 while (millis() - startTime<100) {

 }

 //Serial.printf("dx:%f,dy:%f",dXmm,dYmm);
 //Serial.printf("dx:%i,dy:%i",data.dx,data.dy);
 Serial.printf("x:%f,y:%f,theta:%f",estPosX,estPosY,estYaw);
// Serial.printf("w1:%f,w2:%f,w3:%f,w4:%f,w5:%f,w6:%f,w7:%f,w8:%f",estAngVel[0],estAngVel[1],
// estAngVel[2],estAngVel[3],estAngVel[4],estAngVel[5],estAngVel[6],estAngVel[7]);
 //Serial.printf("x:%f,y:%f",xmm[1],ymm[1]);
 Serial.println();
 }
 c = c + 1;
 }
 
 //delay(10);
}


int convTwosComp(int b){
 //Convert from 2's complement
 //if(sizeof(b) > 
 if(b & 0x80){ // 0x80 (hex) = 10000000 (bin) = 128 (dec)
 b = -1 * ((b ^ 0xff) + 1); // 0xff (hex) = 11111111 (bin) = 255 (dec)
 }
 return b;
}
