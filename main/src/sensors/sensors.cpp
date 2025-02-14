#include "sensors.h"
#include "../config.h"
#include "../globals.h"
#include "../ui/display.h"
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
			measVel[0][i] = -convTwosComp(data[i].dx)*cVal[0][i]/sensingTime;
			measVel[1][i] = convTwosComp(data[i].dy)*cVal[1][i]/sensingTime;

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
	int validVals = 0;
	for (int i = 0; i < 8; i++) {
		if (!isnan(estAngVel[i])) {
			sumAngVel = sumAngVel + estAngVel[i];
			validVals++;
		}
	}

	if (validVals == 0) {
		// Serial.print("Sensors are poo-poo!");
		valid_sensors = false;
		return;
	} else {
		valid_sensors = true;
		estAngVel1 = sumAngVel / validVals;
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
	validVals = 0;
	for (int i = 0; i<ns; i++) {
		if (!isnan(estVel[0][i]) && !isnan(estVel[1][i])) {
			sumVelX = sumVelX + estVel[0][i];
			sumVelY = sumVelY + estVel[1][i];
			validVals++;
		}
	}
	if (validVals == 0) {
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
	for (int i = 0; i < ns; i++) {
		for (int j = 0; j < 2; j++) {
			EEPROM.get(addr, cVal[j][i]);
			addr += sizeof(float);
		}
	}
}

void calibrate() {
	float calDistance = 100.0;
	float calPos[2][3] = {{0.0f, 0.0f, 0.0f},
						  {0.0f, 0.0f, 0.0f}};

	for (int axis = 0; axis < 2; axis++) {
		delay(500);
		calPos[axis][0] = 0.0;

		screen->fillScreen(BLACK);
		drawCenteredText("Move router in direction", 1);
		Serial.printf("Move router in %i direction", axis);

		while (calPos[axis][0] < calDistance) {
			if(micros() - timeLastPoll >= dt) {
				doSensing();
				for (int i = 0; i < ns; i++){
					calPos[axis][i] = calPos[axis][i] + estVel[axis][i]*dt;
				}
			}
			Serial.println(calPos[axis][0]);
		}
		
		screen->fillScreen(BLACK);
		drawCenteredText("STOP", 1);
		delay(100);

		for (int i = 0; i < ns; i++) {
			selfCal[axis][i] = calPos[axis][0]/calPos[axis][i];
		}    
	}

	Serial.println("Calibration values:");
	Serial.printf("\t0: (%f, %f)", selfCal[0][0], selfCal[1][0]);
	Serial.printf("\t1: (%f, %f)", selfCal[0][1], selfCal[1][1]);
	Serial.printf("\t2: (%f, %f)", selfCal[0][2], selfCal[1][2]);
}