#include <EEPROM.h>

// Calibration values (hardcoded for now)
// TODO: serial input or python serial input
// Even better python input    
// MATT VALUES
//0.009558401835 0.009638554217 0.009674922601 0.009652509653 
// -0.009903931861 -0.009958175662 -0.01004520342 -0.009993004897
// CAM VALUES
// 0.00997506234 0.01003310926 0.00996611521 0.01008674602
// 0.01011531459 0.01026588646 0.01019056354 0.01016570093
// float vals[2][4] = {{0.009558401835f, 0.009638554217f, 0.009674922601f,0.009652509653f},
//                     {-0.009903931861f, -0.009958175662f, -0.01004520342f,-0.009993004897f}};
// float vals[2][4] = {{0.00997506234f, 0.01003310926f, 0.00996611521f, 0.01008674602f},
//                     {0.01011531459f, 0.01026588646f, 0.01019056354f, 0.01016570093f}};  // CAM
// float vals[2][4] = {{0.009627418889f, 0.009661462359f, 0.009696311523f, 0.01008674602f},
//                     {0.009927857568f, 0.009981700216f, 0.009993204621f, 0.01016570093f}};  // CAM V2
// float vals[2][4] = {{0.009848093163f, 0.01001452106f, 0.01008954471f, 0.01008674602f},
//                     {0.01005166556f, 0.01023017903f, 0.01029654036f, 0.01016570093f}};  // CAM V3
// float vals[2][4] = {{0.009899225881f, 0.009919060467f, 0.009861932939f, 0.009935617201f},
//                     {0.009997200784f, 0.01003794343f, 0.0100020004f, 0.01003190145f}};  // CAM V4
// float vals[2][4] = {{0.01022411255f, 0.01021297456f, 0.01019077124f, 0.01026483268f},
//                     {0.01031892353f, 0.01044306441f, 0.01036856803f, 0.01044059303f}};  // CAM V5
// float vals[2][4] = {{0.01003579433f, 0.01003237112f, 0.009931012566f, 0.01002318697f},
//                     {0.01040387857f, 0.01025970739f, 0.01015262784f, 0.01023373859f}};  // CAM V6
float vals[2][4] = {{0.009974863344f, 0.01003854802f, 0.01001917001f, 0.01003713741f},
                    {0.01021554806f, 0.01031019954f, 0.01027291717f, 0.01030163178f}};  // CAM V7
      

const int eepromAddrCx = 0;  
const int eepromAddrCy = 12;  

int ns = 4;         // number of sensors in use

// float Cx[3] = {0.0f,0.0f,0.0f};
// float Cy[3] = {0.0f,0.0f,0.0f};
float cVal[2][4] = {{0.0f,0.0f,0.0f,0.0f},
                    {0.0f,0.0f,0.0f,0.0f}}; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  delay(100);

  writeEepromCalibration();

  delay(1000);

  Serial.println("Values written to EEPROM. Reading them back just to double check:");
  readEepromCalibration(cVal);

  Serial.print("Cx values: ");
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < ns; i++) {
      Serial.print(cVal[j][i], 6);
      if (i < ns - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
    if (j == 0){
      Serial.print("Cy values: ");
    }
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}

void writeEepromCalibration() {
  Serial.println("writing calibration values to EEPROM...");


  // int addr = eepromAddrCx;
  // for (int i = 0; i < 3; i++) {
  //   EEPROM.put(addr, x_vals[i]);
  //   addr += sizeof(float);
  // }

  // addr = eepromAddrCy;
  // for (int i = 0; i < 3; i++) {
  //   EEPROM.put(addr, y_vals[i]);
  //   addr += sizeof(float);
  // }

  // Notes
  //  - Calibration values will be stored using (4)x(4)x(2) = 32 bytes
  //  - This comes from (4 bytes per float)x(4 sensors)x(2 vals per sensor)
  //  - The values will be stored in the order (cVal[0][0]),(cVal[1][0]),
  //    (cVal[0][1]),...,(cVal[j][i]), so sensor 0 calibration value for x,
  //    sensor 0 calibration value for y, sensor 1 calibration value for x, and so on
  int addr = 0;         // starting address is always 0
  for (int i = 0; i < ns; i++) {    // sensor number
    for (int j = 0; j < 2; j++) {   // calibration axis (x = 0, y = 1)
      EEPROM.put(addr, vals[j][i]);
      addr += sizeof(float);
    }
  }
}

void readEepromCalibration(float (&cVal)[2][4]) {
  // int addr = eepromAddrCx;
  // for (int i = 0; i < ns; i++) {
  //   Cx[i] = EEPROM.get(addr, Cx[i]);
  //   addr += sizeof(float);
  // }

  // addr = eepromAddrCy;
  // for (int i = 0; i < ns; i++) {
  //   Cy[i] = EEPROM.get(addr, Cy[i]);
  //   addr += sizeof(float);
  // }

  int addr = 0;
  for (int i = 0; i < ns; i++) {
    for (int j = 0; j < 2; j++) {
      EEPROM.get(addr, cVal[j][i]);
      addr += sizeof(float);
    }
  }
}