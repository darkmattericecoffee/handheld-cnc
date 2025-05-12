#include "motors.h"
#include "../config.h"
#include "../globals.h"

void motorSetup() {
	// Set up motors
	pinMode(MOT_EN_R, OUTPUT);
	pinMode(MOT_EN_L, OUTPUT);
	pinMode(MOT_EN_Z, OUTPUT);

	// Enable motors
	digitalWrite(MOT_EN_R, LOW);
	digitalWrite(MOT_EN_L, LOW);
	digitalWrite(MOT_EN_Z, LOW);

	delay(100);

	disableStepperZ();
	// disableStepperRL();

	// Set motor properties
	stepperR.setMinPulseWidth(stepPulseWidth);
	stepperR.setMaxSpeed(zeroSpeed_0);
	stepperR.setAcceleration(zeroAccel);
	stepperR.setCurrentPosition(0);

	stepperL.setMinPulseWidth(stepPulseWidth);
	stepperL.setMaxSpeed(zeroSpeed_0);
	stepperL.setAcceleration(zeroAccel);
	stepperL.setCurrentPosition(0);

	stepperZ.setMinPulseWidth(stepPulseWidth);
	stepperZ.setMaxSpeed(zeroSpeed_0);
	stepperZ.setAcceleration(zeroAccel);
	stepperZ.setCurrentPosition(0);
}

void driverSetup() {
	SERIAL_PORT.begin(115200);
	delay(100);

	driverR.begin(); driverR.toff(5);
	driverL.begin(); driverL.toff(5);
	driverZ.begin(); driverZ.toff(5);

	driverR.rms_current(maxCurrent_RMS);
	driverL.rms_current(maxCurrent_RMS);
	driverZ.rms_current(maxCurrent_RMS);
	
	driverR.microsteps(uSteps);
	driverL.microsteps(uSteps);
	driverZ.microsteps(uSteps);

	driverR.pwm_autoscale(true);
	driverR.en_spreadCycle(false);
	driverR.TPWMTHRS(0x753);

	driverL.pwm_autoscale(true);
	driverL.en_spreadCycle(false);
	driverL.TPWMTHRS(0x753);
	
	driverZ.pwm_autoscale(true);
	driverZ.en_spreadCycle(false);
	driverZ.TPWMTHRS(0x753);
}

void enableStepperZ() {
	digitalWrite(MOT_EN_Z, LOW);
}

void enableStepperRL() {
	digitalWrite(MOT_EN_R, LOW);
	digitalWrite(MOT_EN_L, LOW);
}

void disableStepperZ() {
	digitalWrite(MOT_EN_Z, HIGH);
}

void disableStepperRL() {
	digitalWrite(MOT_EN_R, HIGH);
	digitalWrite(MOT_EN_L, HIGH);
}

void stopStepperX() {
	stepperR.setSpeed(0);
	stepperL.runSpeed();
}

void stopStepperZ() {
	stepperZ.setSpeed(0);
	stepperZ.runSpeed();
}

void machineZeroXY() {
	// TODO: adjust this zeroing procedure
	// This zeroing process is just using the zeroing plug for now (no limit switches)
	pose = {0.0f};
	enableStepperRL();
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
	desPos.set(0.0f, 0.0f, 0.0f);
	actuate(desPos);

	while (stepperR.distanceToGo() != 0 && stepperL.distanceToGo() != 0) {
		stepperR.run();
		stepperL.run();
	}

	stepperZ.moveTo(Conv*restHeight);
	while (stepperZ.distanceToGo() != 0) {
		stepperZ.run();
	}

	// Reset router pose
	pose = {0.0f};
}

void actuate(Position pos) {
	// actuate coreXY system
	float a = pos.getX() + pos.getY();
	float b = pos.getX() - pos.getY();
	stepperR.moveTo(a * ConvBelt);
	stepperL.moveTo(b * ConvBelt);
	stepperZ.moveTo(pos.getZ() * Conv);
}