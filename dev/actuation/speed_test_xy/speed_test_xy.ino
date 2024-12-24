#include <AccelStepper.h>
#include <TMCStepper.h>
#include <EncoderButton.h>

// Pin definitions
#define LIMIT_MACH_X0       6
#define LIMIT_MACH_Z0       5
// #define LIMIT_MACH_X0       5
// #define LIMIT_MACH_Z0       6
#define BUTT_HANDLE_L       2
#define BUTT_HANDLE_R       3
#define ENCODER_PIN_A       21
#define ENCODER_PIN_B       22
#define ENCODER_BUTTON_PIN  4
#define MOT_EN_X    		18
#define MOT_DIR_X   		14
#define MOT_STEP_X  		15
#define MOT_EN_Z    		41
#define MOT_DIR_Z   		36
#define MOT_STEP_Z  		33
#define TFT_CS      		31
#define TFT_DC      		30
#define TFT_RST     		-1

// Driver settings
#define DRIVER_ADDRESS      0b00
#define R_SENSE            	0.11f
#define SERIAL_PORT_X     	Serial4
#define SERIAL_PORT_Z     	Serial8

// Constants
#define MAX_PATHS  			10
#define MAX_POINTS 			1000
#define NUM_DESIGNS 		7
#define GC9A01A_WEBWORK_GREEN 0x8FF1
#define motorInterfaceType 	1

// Motor properties
const int uSteps = 4;							// microstep configuration
const float lead = 8;							// lead screw lead (mm)
const float Conv = 200*uSteps/lead;				// conversion factor (mm -> steps)
const float stepPulseWidth = 20.0;				// min pulse width (from Mark Rober's code)
const float maxCurrent_RMS = 1273.0;			// motor RMS current rating (mA)
const float maxSpeedX = 200.0*Conv;				// max velocity X motor can move at (step/s)
const float maxSpeedZ = 100.0*Conv;				// max velocity Z motor can move at (step/s)
const float maxAccelX = 3000.0*Conv;				// max acceleration (mm/s^2 -> step/s^2)
const float maxAccelZ = 3000.0*Conv;        // max acceleration (mm/s^2 -> step/s^2)
const float retract = 5;						// distance to retract (mm)
const float zeroSpeed_0 = 20.0 * Conv;				// x zeroing speed (step/s)
const float zeroSpeed_1 = 4.0 * Conv;				// x secondary zeroing speed (step/s)
const float zeroAccel = 200.0 * Conv;			// x zeroing acceleration (step/s^2)

// Material properties
const float maxThickness = 15.0;				// upper bound of thickness knob (mm)
const float restHeight = 2.0;					// rest height of tool before cutting (mm)

// Gantry geometry
// const float gantryLength = 106.0;				// usable length of x-gantry (mm)
const float gantryLength = 34.0;				// usable length of x-gantry (mm) (FOR XZ SWAP TESTING)
const float xLimitOffset = 2.54;				// distance from wall of stepper when zeroed (mm)
const float xBuffer = 3.0;						// safety buffer between tool body and walls (mm)
const float xSensorOffset = -2.32;				
const float ySensorOffset = -3.2;				// offset to try to counteract weird rotation behavior (mm) (UNUSED)
const float zLength = 34.0;						// usable length of z-gantry (mm)
const float zLimitOffset = 2.13;				// distance from wall when zeroed (mm)

// Timing constants
const long unsigned debounceDelay = 50;			// the debounce time; increase if the output flickers
const long unsigned dtDebug = 500;				// (ms)
const long unsigned dtPlot = 50;				// (ms)
const long unsigned dtOutput = 20;				// (ms)
const long unsigned dtOutputSD = 10;			// (ms)
const long unsigned dt = 900;					// length of sensor timestep (us)(freq = 1,000,000/timestepPoll [Hz])

// Test parameters
float currentPos = 0.0;               // (mm)
float currentSpeed = 0;               // (steps/s)
float speedIncrement = 1.0 * Conv;   // (steps/s)
float currentAccel = maxAccelZ/2;
bool testing_x = true;
bool emergency_stop = false;
unsigned long lastSpeedUpdate = 0;
const unsigned long speedUpdateInterval = 1;  // ms

// Initialize hardware objects
AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);
TMC2209Stepper driverX(&SERIAL_PORT_X, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverZ(&SERIAL_PORT_Z, R_SENSE, DRIVER_ADDRESS);
EncoderButton encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_BUTTON_PIN);

void setup() {
	Serial.begin(115200);
	motorSetup();
	driverSetup();

  // Set up limit switch pins
	pinMode(LIMIT_MACH_X0, INPUT);
	pinMode(LIMIT_MACH_Z0, INPUT);
	
	// Zero both axes
	machineZeroX();
	machineZeroZ();

  // encoder.setClickHandler(onClickZeroMachineX);
	// encoder.setTripleClickHandler(onClickGoToSetThickness);
	// onClickResetState(encoder);
  encoder.setEncoderHandler(onEncoderUpdateSpeed);
	
	Serial.println("Motor Test Ready");
	Serial.println("Use these commands:");
	Serial.println("s: Start/Stop test");
	Serial.println("x/y: Toggle axis");
	Serial.println("+/-: Adjust speed");
	Serial.println("e: Emergency stop");

}

void loop() {
	if (Serial.available() > 0) {
		char cmd = Serial.read();
		handleCommand(cmd);
	}

  encoder.update();
	
	if (!emergency_stop && currentSpeed != 0) {
		unsigned long currentTime = millis();
		if (currentTime - lastSpeedUpdate >= speedUpdateInterval) {
			updateMotorPosition();
			lastSpeedUpdate = currentTime;
		}
	}

  
}

void motorSetup() {
	// Set up motors
	pinMode(MOT_EN_X, OUTPUT);
	pinMode(MOT_EN_Z, OUTPUT);

	// Enable motors
	digitalWrite(MOT_EN_X, LOW);
	digitalWrite(MOT_EN_Z, LOW);

	delay(100);
	// disableStepperZ();

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
	while (digitalRead(LIMIT_MACH_X0) == HIGH) { stepperX.runSpeed(); }

	stepperX.move(-Conv*retract);
	while (stepperX.distanceToGo() != 0) { stepperX.run(); }

	stepperX.setSpeed(zeroSpeed_1);
	while (digitalRead(LIMIT_MACH_X0) == HIGH) { stepperX.runSpeed(); }

	stepperX.setMaxSpeed(maxSpeedX/2);
	stepperX.setAcceleration(maxAccelX/2);

	stepperX.move(-Conv*((gantryLength/2) - xLimitOffset));
	while (stepperX.distanceToGo() != 0) { stepperX.run(); }

	stepperX.setMaxSpeed(maxSpeedX);
	stepperX.setAcceleration(maxAccelX);
	stepperX.setCurrentPosition(0);
}

void machineZeroZ() {
  stepperZ.setSpeed(zeroSpeed_0);
	while (digitalRead(LIMIT_MACH_Z0) == HIGH) { stepperZ.runSpeed(); }

	stepperZ.move(-Conv*retract);
	while (stepperZ.distanceToGo() != 0) { stepperZ.run(); }

	stepperZ.setSpeed(zeroSpeed_1);
	while (digitalRead(LIMIT_MACH_Z0) == HIGH) { stepperZ.runSpeed(); }

	stepperZ.setMaxSpeed(maxSpeedX/2);
	stepperZ.setAcceleration(maxAccelZ/2);

	stepperZ.move(-Conv*((zLength/2) - zLimitOffset));
	while (stepperZ.distanceToGo() != 0) { stepperZ.run(); }    // ***skipping steps here***

	stepperZ.setMaxSpeed(maxSpeedX);
	stepperZ.setAcceleration(maxAccelZ);
	stepperZ.setCurrentPosition(0);
}

void handleCommand(char cmd) {
	switch (cmd) {
		case 's':
			toggleTest();
			break;
		case 'x':
			testing_x = true;
			currentSpeed = 0;
			Serial.println("Testing X axis");
			break;
		case 'y':
			testing_x = false;
			currentSpeed = 0;
			Serial.println("Testing Z axis");
			break;
		case '+':
			incrementSpeed();
			break;
		case '-':
			decrementSpeed();
			break;
		case 'e':
			emergencyStop();
			break;
	}
}

void toggleTest() {
	if (currentSpeed == 0) {
		if (testing_x) {
			stepperX.setMaxSpeed(maxSpeedX);
			stepperX.setAcceleration(maxAccelX);
			currentSpeed = zeroSpeed_0;
		} else {
			stepperZ.setMaxSpeed(maxSpeedZ);
			stepperZ.setAcceleration(maxAccelZ);
			currentSpeed = zeroSpeed_0;
		}
		emergency_stop = false;
		Serial.println("Test started");
	} else {
		currentSpeed = 0;
		stopMotors();
		Serial.println("Test stopped");
	}
}

void nullHandler(EncoderButton &eb) {
	Serial.println("null handler called");
	return;
}

void onEncoderUpdateSpeed(EncoderButton &eb) {
	float tempSpeed = currentSpeed + eb.increment()*speedIncrement;

	if (tempSpeed <= maxSpeedX) {
		currentSpeed = tempSpeed;
	}

  Serial.printf("Speed = %f mm/s\n", currentSpeed/Conv);
	
	// char text2send[50];
	// sprintf(text2send, "Turn to set thickness\n%.2f mm", matThickness);
	// drawCenteredText(text2send, 1);
}

void incrementSpeed() {
	if (testing_x && currentSpeed < maxSpeedX) {
		currentSpeed += speedIncrement;
	} else if (!testing_x && currentSpeed < maxSpeedZ) {
		currentSpeed += speedIncrement;
	}
	Serial.print("Speed: ");
	Serial.println(currentSpeed/Conv);
}

void decrementSpeed() {
	if (currentSpeed > speedIncrement && currentSpeed > 0) {
		currentSpeed -= speedIncrement;
		Serial.print("Speed: ");
		Serial.println(currentSpeed/Conv);
	}
}

void updateMotorPosition() {
	if (testing_x) {
		currentPos = stepperX.currentPosition()/Conv;
		if (currentPos <= xBuffer) {
			stepperX.setSpeed(currentSpeed);
		} else if (currentPos >= gantryLength - xBuffer) {
			stepperX.setSpeed(-currentSpeed);
		}
		stepperX.runSpeed();
	} else {
		currentPos = stepperZ.currentPosition()/Conv;
		if (currentPos <= zLimitOffset) {
			stepperZ.setSpeed(currentSpeed);
		} else if (currentPos >= zLength - zLimitOffset) {
			stepperZ.setSpeed(-currentSpeed);
		}
		stepperZ.runSpeed();
	}
}

void emergencyStop() {
	emergency_stop = true;
	currentSpeed = 0;
	stopMotors();
	Serial.println("EMERGENCY STOP");
}

void stopMotors() {
	stopStepperX();
	stopStepperZ();
}