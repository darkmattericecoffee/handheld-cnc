#include "sensors.h"
#include "../config.h"
#include "../globals.h"
#include "../ui/display.h"
#include "../ui/encoder.h"
#include "../math/geometry.h"
#include <EEPROM.h>

/*
Sensor configuration:
0 --- 1    ^ y
|  x  |    |-> x
2 --- 3

Angle signage: +CCW
*/

// Timing variables
static long unsigned timeLastPlot = 0;

// Additional tracking variables
static float estPosSen[2][4] = {{0.0f,0.0f,0.0f,0.0f},
							   {0.0f,0.0f,0.0f,0.0f}};
static int surfaceQuality[4] = {0,0,0,0};
static float estVel[2][4] = {{0.0f,0.0f,0.0f,0.0f},
							{0.0f,0.0f,0.0f,0.0f}};
static float estAngVel[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

void sensorSetup() {
	int sensorPins[4] = {SS0, SS1, SS2, SS3};
	
	for (int i = 0; i < 4; i++) {
		if (sensors[i].begin(sensorPins[i], CPI)) {
			Serial.print("Sensor");
			Serial.print(i);
			Serial.print(" initialization succeeded, with CPI = ");
			Serial.println(sensors[i].getCPI());
		} else {
			Serial.print("Sensor");
			Serial.print(i);
			Serial.println(" initialization failed");
		}
	}
}

int16_t convTwosComp(int16_t value) {
	if (value & 0x8000) {
		return -((~value & 0xFFFF) + 1);
	} else {
		return value;
	}
}

void doSensing() {
	timeLastPoll = micros();

	// Collect sensor data
	PMW3360_DATA data[ns];
	for (int i = 0; i < ns; i++) {
		data[i] = sensors[i].readBurst();
	}

	// Process sensor data
	for (int i = 0; i < ns; i++) {
		surfaceQuality[i] = data[i].SQUAL;
		if (surfaceQuality[i] > 20) {
			float dx = -convTwosComp(data[i].dx);
			float dy = convTwosComp(data[i].dy);
			measVel[0][i] = dx*cVal[0][i] / sensingTime;
			measVel[1][i] = dy*cVal[1][i] / sensingTime;

			estPosSen[0][i] = estPosSen[0][i] + measVel[0][i]*sensingTime;
			estPosSen[1][i] = estPosSen[1][i] + measVel[1][i]*sensingTime;
		} else {
			measVel[0][i] = NAN;
			measVel[1][i] = NAN;
		}
	}

	// Calculate angular velocities
	estAngVel[0] = (measVel[0][2] - measVel[0][0])/ly;
	estAngVel[1] = (measVel[0][2] - measVel[0][1])/ly;
	estAngVel[2] = (measVel[0][3] - measVel[0][0])/ly;
	estAngVel[3] = (measVel[0][3] - measVel[0][1])/ly;
	estAngVel[4] = (measVel[1][1] - measVel[1][0])/lx;
	estAngVel[5] = (measVel[1][1] - measVel[1][2])/lx;
	estAngVel[6] = (measVel[1][3] - measVel[1][0])/lx;
	estAngVel[7] = (measVel[1][3] - measVel[1][2])/lx;

	// Average angular velocities
	float sumAngVel = 0.0f;
	float estAngVel1 = 0.0f;
	int valalScalars = 0;
	for (int i = 0; i < 8; i++) {
		if (!isnan(estAngVel[i])) {
			sumAngVel = sumAngVel + estAngVel[i];
			valalScalars++;
		}
	}

	if (valalScalars == 0) {
		// Serial.print("Sensors are poo-poo!");
		valid_sensors = false;
		return;
	} else {
		valid_sensors = true;
		estAngVel1 = sumAngVel / valalScalars;
	}

	// Body position estimation
	estVel[0][0] = measVel[0][0]*cosf(pose.yaw)-measVel[1][0]*sinf(pose.yaw) + 0.5*estAngVel1*(lx*cosf(pose.yaw)-(ly)*sinf(pose.yaw));
	estVel[0][1] = measVel[0][1]*cosf(pose.yaw)-measVel[1][1]*sinf(pose.yaw) + 0.5*estAngVel1*(lx*cosf(pose.yaw)+(ly)*sinf(pose.yaw));
	estVel[0][2] = measVel[0][2]*cosf(pose.yaw)-measVel[1][2]*sinf(pose.yaw) + 0.5*estAngVel1*(-lx*cosf(pose.yaw)-(ly)*sinf(pose.yaw));
	estVel[0][3] = measVel[0][3]*cosf(pose.yaw)-measVel[1][3]*sinf(pose.yaw) + 0.5*estAngVel1*(-lx*cosf(pose.yaw)+(ly)*sinf(pose.yaw));
	estVel[1][0] = measVel[0][0]*sinf(pose.yaw)+measVel[1][0]*cosf(pose.yaw) + 0.5*estAngVel1*((ly)*cosf(pose.yaw)+lx*sinf(pose.yaw));
	estVel[1][1] = measVel[0][1]*sinf(pose.yaw)+measVel[1][1]*cosf(pose.yaw) + 0.5*estAngVel1*(-(ly)*cosf(pose.yaw)+lx*sinf(pose.yaw));
	estVel[1][2] = measVel[0][2]*sinf(pose.yaw)+measVel[1][2]*cosf(pose.yaw) + 0.5*estAngVel1*((ly)*cosf(pose.yaw)-lx*sinf(pose.yaw));
	estVel[1][3] = measVel[0][3]*sinf(pose.yaw)+measVel[1][3]*cosf(pose.yaw) + 0.5*estAngVel1*(-(ly)*cosf(pose.yaw)-lx*sinf(pose.yaw));

	// Simple average of linear velocities
	float sumVelX = 0.0f;
	float sumVelY = 0.0f;
	float estVel1[2] = {0.0f, 0.0f};
	valalScalars = 0;
	for (int i = 0; i<ns; i++) {
		if (!isnan(estVel[0][i]) && !isnan(estVel[1][i])) {
			sumVelX = sumVelX + estVel[0][i];
			sumVelY = sumVelY + estVel[1][i];
			valalScalars++;
		}
	}
	if (valalScalars == 0) {
		// TODO: make this prompt a re-zeroing
		// Serial.print("Sensors are poo-poo!");
		valid_sensors = false;
		return;
	} else {
		valid_sensors = true;
		// (TODO: figure out deeper cause) Acounting for weird rotation
		// estVel1[0] = (sumVelX / ns) - estAngVel1*(xSensorOffset*sinf(pose.yaw) + ySensorOffset*cosf(pose.yaw));
		// estVel1[1] = (sumVelY / ns) + estAngVel1*(xSensorOffset*cosf(pose.yaw) - ySensorOffset*sinf(pose.yaw));
		estVel1[0] = sumVelX / ns;
		estVel1[1] = sumVelY / ns;
	}

	// Integrate to get position and orientation
	pose.yaw = pose.yaw + estAngVel1*sensingTime;
	pose.x = pose.x + estVel1[0]*sensingTime;
	pose.y = pose.y + estVel1[1]*sensingTime;

	// Sensor plotting
	if (plottingOn) {
		sensorPlotting();
	}
}

void doSensingLinear() {
    timeLastPoll = micros();

    // Sensing ---------------------------------------------------------------------
    // Collect sensor data (raw)
 
    PMW3360_DATA data[4];
    for (int i = 0; i < 4; i++) {
      data[i] = sensors[i].readBurst();
      surfaceQuality[i] = data[i].SQUAL;
    }

    for (int i = 0; i < 4; i++) {
      // Sensor velocity sensing
      // measVel[0][i] = -convTwosComp(data[i].dx);     // '-' convention is used to flip sensor's z axis
      measVel[0][i] = -convTwosComp(data[i].dx);
      measVel[1][i] = convTwosComp(data[i].dy);

      // Integrate linear velocities to get position
      calPos[0][i] = calPos[0][i] + measVel[0][i];
      calPos[1][i] = calPos[1][i] + measVel[1][i];
    }

    // Sensor plotting
	for (int i = 0; i < 4; i++) {
		float angle = atanf(calPos[1][i]/calPos[0][i]);
		Serial.printf("%i: x:%.2f,\ty:%.2f,\tb:%.3f",i,calPos[0][i],calPos[1][i],angle);
		Serial.println();
	}
}

void sensorPlotting() {
	if(millis() - timeLastPlot >= dtPlot) {
		timeLastPlot = millis();

		for (int i = 0; i < 4; i++) {
			Serial.printf("x_%i:%f,y_%i:%f",i,estPosSen[0][i],i,estPosSen[1][i]);
			Serial.println();
			Serial.printf("sq_%i:%d",i,surfaceQuality[i]);
			Serial.println();
		}
	}
}

void readEepromCalibration(float (&cVal)[2][4]) {
	int addr = 0;
	float tempVal = 0.0f;
	for (int i = 0; i < ns; i++) {
		for (int j = 0; j < 2; j++) {
			EEPROM.get(addr, tempVal);

			// check if calibration has been performed
			cVal[j][i] = isnan(tempVal) ? cVal[j][i] : tempVal;

			addr += sizeof(float);
		}
	}
}

void calibrate() {
	encoder.setEncoderHandler(nullHandler);
	encoder.setClickHandler(onClickCalibrationAdvance);

	float calDistance = 300.0;
	int currentRun = 0;
	int numRuns = 5;
	float tempCalScalar[2][ns] = {0.0f};
	float tempCalRotation[2][ns] = {0.0f};

	for (int axis = 0; axis < 2; axis++) {
		while (currentRun < numRuns) {
			if (axis == 0) {
				drawCenteredText("Align X axis\nClick when at 0", 2);
			} else {
				drawCenteredText("Align Y axis\nClick when at 0", 2);
			}
			while (state != CALIBRATION_ADVANCE) encoder.update();
			state = CALIBRATION;
			
			// Flush sensor readings then reset
			for (int i = 0; i < 4; i++) {
				sensors[i].readBurst();
				calPos[0][i] = 0.0f;
				calPos[1][i] = 0.0f;
			}

			// prompt x calibration
			// 	- do x run
			//		- "Reached 300?"
			// 		- "Retry?"
			if (axis == 0) {
				drawCenteredText("Move in +X\nClick when at 300mm", 2);
			} else {
				drawCenteredText("Move in +Y\nClick when at 300mm", 2);
			}
			while (state != CALIBRATION_ADVANCE) {
				doSensingLinear();
				encoder.update();
			}
			
			for (int i = 0; i < ns; i++) {
				float sensorDist = calDistance / (myDist(calPos[0][i],calPos[1][i],0,0));
				tempCalScalar[axis][i] = tempCalScalar[axis][i] + (sensorDist / numRuns);

				float sensorRot = axis ? atanf(-calPos[0][i]/calPos[1][i]) : atanf(calPos[1][i]/calPos[0][i]);
				tempCalRotation[axis][i] =  tempCalRotation[axis][i] + (sensorRot / numRuns);
			}

			state = CALIBRATION;
			currentRun++;
		}
		currentRun = 0;
	}

	Serial.println("Calibration results:");
	for (int i = 0; i < ns; i++) {
		float avgRot = (tempCalRotation[0][i]+tempCalRotation[1][i]) / 2;
		Serial.printf("\tCx:%f,Cy:%f,Cr:%f\n",tempCalScalar[0][i],tempCalScalar[1][i],avgRot);
	}



}