#include "motors.h"
#include "../config.h"
#include "../globals.h"

void motorSetup() {
	// Set up motors
	pinMode(MOT_EN_X, OUTPUT);
	pinMode(MOT_EN_Z, OUTPUT);

	// Enable motors
	digitalWrite(MOT_EN_X, LOW);
	digitalWrite(MOT_EN_Z, LOW);

	delay(100);
	disableStepperZ();

	// Set motor properties
	stepperX.setMinPulseWidth(stepPulseWidth);
	stepperX.setMaxSpeed(zeroSpeed_0);
	stepperX.setAcceleration(zeroAccel);
	stepperX.setCurrentPosition(0);
	
	stepperZ.setMinPulseWidth(stepPulseWidth);
	stepperZ.setMaxSpeed(zeroSpeed_0);
	stepperZ.setAcceleration(zeroAccel);
	stepperZ.setCurrentPosition(0);
}

void driverSetup() {
	SERIAL_PORT_X.begin(115200);
	SERIAL_PORT_Z.begin(115200);
	delay(100);

	driverX.begin();
	driverX.toff(5);
	driverZ.begin();
	driverZ.toff(5);

	driverX.rms_current(maxCurrent_RMS);
	driverZ.rms_current(maxCurrent_RMS);
	
	if (uSteps > 1) {
		driverX.microsteps(uSteps);
		driverZ.microsteps(uSteps);
	} else {
		driverX.microsteps(0);
		driverZ.microsteps(0);
	}

	driverX.pwm_autoscale(true);
	driverX.en_spreadCycle(false);
	driverX.TPWMTHRS(0x753);
	
	driverZ.pwm_autoscale(true);
	driverZ.en_spreadCycle(false);
	driverZ.TPWMTHRS(0x753);
}

void enableStepperZ() {
	digitalWrite(MOT_EN_Z, LOW);
}

void disableStepperZ() {
	digitalWrite(MOT_EN_Z, HIGH);
}

void stopStepperX() {
	stepperX.setSpeed(0);
	stepperX.runSpeed();
}

void stopStepperZ() {
	stepperZ.setSpeed(0);
	stepperZ.runSpeed();
}

void machineZeroX() {
	stepperX.setSpeed(zeroSpeed_0);
	while (digitalRead(LIMIT_MACH_X0) == HIGH) {
		stepperX.runSpeed();
	}

	stepperX.move(-Conv*retract);
	while (stepperX.distanceToGo() != 0) {
		stepperX.run();
	}

	stepperX.setSpeed(zeroSpeed_1);
	while (digitalRead(LIMIT_MACH_X0) == HIGH) {
		stepperX.runSpeed();
	}

	stepperX.setMaxSpeed(maxSpeedX/2);
	stepperX.setAcceleration(maxAccelX/2);

	stepperX.move(-Conv*((gantryLength/2) - xLimitOffset));
	while (stepperX.distanceToGo() != 0) {
		stepperX.run();
	}

	stepperX.setMaxSpeed(maxSpeedX);
	stepperX.setAcceleration(maxAccelX);
	stepperX.setCurrentPosition(0);
}

void workspaceZeroZ() {
	enableStepperZ();
	stepperZ.setCurrentPosition(0);

	stepperZ.setSpeed(zeroSpeed_0);
	while (digitalRead(LIMIT_MACH_Z0) == HIGH) {
		stepperZ.runSpeed();
	}

	stepperZ.move(-Conv*retract);
	while (stepperZ.distanceToGo() != 0) {
		stepperZ.run();
	}

	stepperZ.setSpeed(zeroSpeed_1);
	while (digitalRead(LIMIT_MACH_Z0) == HIGH) {
		stepperZ.runSpeed();
	}

	float maxHeight = stepperZ.currentPosition()*1.0f / Conv;

	stepperZ.setMaxSpeed(maxSpeedZ/2);
	stepperZ.moveTo(Conv*restHeight);
	while (stepperZ.distanceToGo() != 0) {
		stepperZ.run();
	}

	stepperZ.setMaxSpeed(maxSpeedZ);
	stepperZ.setAcceleration(maxAccelZ);
}

void workspaceZeroXY() {
	stepperX.moveTo(0);
	while (stepperX.distanceToGo() != 0) {
		stepperX.run();
	}

	stepperZ.moveTo(Conv*restHeight);
	while (stepperZ.distanceToGo() != 0) {
		stepperZ.run();
	}

	// Reset router pose
	pose = {0.0f};
}
