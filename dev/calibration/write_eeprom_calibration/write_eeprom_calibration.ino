#include <EEPROM.h>

// Calibration parameters
typedef struct CalParams {
	float x = 0.01f;		// x scalar
	float y = 0.01f;		// y scalar
	float r = 0.0f;			// sensor rotation
} CalParams;

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
float vals[3][4] = {{0.009974863344f, 0.01003854802f, 0.01001917001f, 0.01003713741f},
                    {0.01021554806f,  0.01031019954f, 0.01027291717f, 0.01030163178f},
                    {0.0f,            0.0f,           0.0f,           0.0f}};  // CAM V7
      

int ns = 4;         // number of sensors in use

CalParams cal[4];

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);

	delay(100);

	writeEepromCalibration();

	delay(1000);

	Serial.println("Values written to EEPROM. Reading them back just to double check:");
	readEepromCalibration();

	for (int i = 0; i < ns; i++) {
		Serial.printf("Sensor %i:\tCx:%.4f, Cy:%.4f, Cr:%.4f\n", i, cal[i].x, cal[i].y, cal[i].r);
	}

}

void loop() {
  // put your main code here, to run repeatedly:

}

void writeEepromCalibration() {
	//  - Calibration values will be stored using (4)x(4)x(3) = 48 bytes
	//  - This comes from (4 bytes per float)x(4 sensors)x(2 vals per sensor)
	//	- The values will be stored in the order (cal[0].x),(cal[0].y),(cal[0].r),
	//		(cal[1].x),...,(cal[i].r), so sensor 0 calibration value for x,
	//		sensor 0 calibration value for y, sensor 0 calibration for rotation,
	//		sensor 1 calibration value for x, and so on

	Serial.println("writing calibration values to EEPROM...");

	int addr = 0;         // starting address is always 0
	for (int i = 0; i < ns; i++) {    // sensor number
		EEPROM.put(addr, vals[0][i]);
		addr += sizeof(float);
		EEPROM.put(addr, vals[1][i]);
		addr += sizeof(float);
		EEPROM.put(addr, vals[2][i]);
		addr += sizeof(float);
	}
}

void readEepromCalibration() {
	int addr = 0;
	float tempVal = 0.0f;
	for (int i = 0; i < ns; i++) {
		EEPROM.get(addr, tempVal);
		cal[i].x = isnan(tempVal) ? cal[i].x : tempVal;			// check if calibration has been performed
		addr += sizeof(float);

		EEPROM.get(addr, tempVal);
		cal[i].y = isnan(tempVal) ? cal[i].y : tempVal;
		addr += sizeof(float);

		EEPROM.get(addr, tempVal);
		cal[i].r = isnan(tempVal) ? cal[i].r : tempVal;
		addr += sizeof(float);
	}
}