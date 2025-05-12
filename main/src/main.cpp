#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "globals.h"
#include "motors/motors.h"
#include "sensors/sensors.h"
#include "ui/display.h"
#include "ui/encoder.h"
#include "path/path-generators.h"
#include "path/path-execution.h"
#include "math/geometry.h"
#include "io/logging.h"

void setup() {
	Serial.begin(115200);  

	if (outputOn) { while(!Serial); }
	delay(100);

	if (!screen->begin()) { Serial.println("screen->begin() failed!"); }

	screen->fillScreen(BLACK);

	radius = screen->width()*0.95 / 4;
	centerX = screen->width() / 2;
	centerY = screen->width() / 2;

	drawCenteredText("Initializing...", 2);
	delay(200);

	Serial.println("Loading calibration coefficients:");
	readEepromCalibration();

	for (int i = 0; i < ns; i++) {
		Serial.printf("Sensor %i:\tCx:%.4f, Cy:%.4f, Cr:%.4f\n", i, cal[i].x, cal[i].y, cal[i].r);
	}

	// Initialize buttons
	pinMode(LIMIT_MACH_X0, INPUT);
	pinMode(LIMIT_MACH_Z0, INPUT);
	pinMode(BUTT_HANDLE_L, INPUT);
	pinMode(BUTT_HANDLE_R, INPUT);

	// Setup systems
	sensorSetup();
	motorSetup(); 
	driverSetup();
	
	Serial.print("Initializing SD card...");
	if (!sd.begin(SdioConfig(FIFO_SDIO))) {
		Serial.println("Initialization failed!");
		return;
	}
	Serial.println("Initialization done.");

	encoder.setClickHandler(onClickZeroMachineXY);
	encoder.setTripleClickHandler(onClickGoToSetThickness);
	// onClickResetState(encoder);
}

void loop() {
	timeLoopStart = micros();

	// Sensing
	unsigned long startSensingTime = micros();
    if(micros() - timeLastPoll >= dt) {
        sensingTime = micros() - timeLastPoll;
        doSensing();
    }
	sensingTime_debug = micros() - startSensingTime;

	// Run steppers
	unsigned long startStepperTime = micros();
	stepperR.run();
	stepperL.run();
	stepperZ.run();
	encoder.update();
	stepperTime = micros() - startStepperTime;

	// Serial handling
	unsigned long startSerialTime = micros();
	handleSerial();
	serialTime = micros() - startSerialTime;

	// --Break here until we are ready to cut--
	// if (state != READY) {
	// 	totalLoopTime = micros() - timeLoopStart;
	// 	return;
	// }

	// Safety stuff
	unsigned long startSafetyTime = micros();
	if (!checkEndstops()) {
		safetyTime = micros() - startSafetyTime;
		totalLoopTime = micros() - timeLoopStart;
		return;
	}
	safetyTime = micros() - startSafetyTime;

	// Cutting
	unsigned long startCuttingTime = micros();
	handleChickenHead();
	// handleCutting();
	cuttingTime = micros() - startCuttingTime;

	totalLoopTime = micros() - timeLoopStart;

	// Debugging
	if (stopwatchOn) {
		stopwatch();
	}
}