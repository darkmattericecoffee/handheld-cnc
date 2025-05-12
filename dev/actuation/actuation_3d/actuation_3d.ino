#include <AccelStepper.h>
#include <TMCStepper.h>
// #include <PMW3360.h>
#include <SdFat.h>
#include <Arduino_GFX_Library.h>
#include <EncoderButton.h>

// Pin definitions
#define SS0   				30
#define SS1					10
#define SS2					31
#define SS3					32
#define LIMIT_MACH_X0       6
#define LIMIT_MACH_Z0       5
#define BUTT_HANDLE_L       2
#define BUTT_HANDLE_R       3
#define ENCODER_PIN_A       22
#define ENCODER_PIN_B       21
#define ENCODER_BUTT	  	4
#define MOT_EN_R    		23
#define MOT_DIR_R   		18
#define MOT_STEP_R  		19
#define MOT_EN_L    		15
#define MOT_DIR_L   		38
#define MOT_STEP_L  		39
#define MOT_EN_Z    		41
#define MOT_DIR_Z   		36
#define MOT_STEP_Z  		33
#define TFT_CS      		29
#define TFT_DC      		28
#define TFT_RST     		25
#define TFT_BL				24

// Driver settings
#define DRIVER_ADDRESS_R	0b00
#define DRIVER_ADDRESS_L  	0b01
#define DRIVER_ADDRESS_Z	0b10
#define R_SENSE            	0.11f
#define SERIAL_PORT     	Serial4
#define motorInterfaceType 	1

// Motor properties
const int stepsPerRev = 200;					// steps per revolution
const int uSteps = 4;							// microstep configuration
// A and B Motors (belt)
const float beltPitch = 2.0;					// belt pitch (mm)
const int pulleyTeeth = 16;						// pulley teeth on drive pulley
const float ConvBelt = (stepsPerRev*uSteps)/(beltPitch*pulleyTeeth);	// conversion factor (mm -> steps)
const float maxSpeedAB = 140.0*ConvBelt;			// max velocity A and B motors can move at (step/s)
const float maxAccelAB = 2000.0*ConvBelt;			// A/B motor max acceleration (step/s^2)
// Z Motor (lead screw)
const float lead = 8;							// lead screw lead (mm)
const float Conv = (stepsPerRev*uSteps)/lead;	// conversion factor (mm -> steps)
const float stepPulseWidth = 20.0;				// min pulse width (from Mark Rober's code)
const float maxCurrent_RMS = 1273.0;			// motor RMS current rating (mA)
const float maxSpeedZ = 180.0*Conv;				// max velocity Z motor can move at (step/s)
const float maxAccelZ = 3000.0*Conv;			// z max acceleration (step/s^2)
const float retract = 5;						// distance to retract (mm)
const float zeroSpeed_0 = 20.0 * Conv;			// zeroing speed (step/s)
const float zeroSpeed_1 = 4.0 * Conv;			// secondary zeroing speed (step/s)
const float zeroAccel = 200.0 * Conv;			// zeroing acceleration (step/s^2)
const float holeFeedrate = 5.0*Conv; 			// default hole feedrate (step/s)		TODO: make this modifiable

// Gantry geometry
const float xRange = 20.0;						// usable range of x-axis (mm)
const float yRange = 20.0;						// usable range of y-axis (mm)
const float zRange = 34.0;						// usable length of z-axis (mm)
const float xLimitOffset = 2.54;				// distance from wall of stepper when zeroed (mm)
const float zLimitOffset = 2.13;				// distance from wall when zeroed (mm)
const float wallBuffer = 3.0;						// safety buffer between tool body and walls (mm)

class Position {
public:
	Position(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {
		clamp();
	}

	void set(float newX, float newY, float newZ) {
		x = newX;
		y = newY;
		z = newZ;
		clamp();
	}

	void setX(float newX) {
		x = newX;
		clamp();
	}

	void setY(float newY) {
		y = newY;
		clamp();
	}

	void setZ(float newZ) {
		z = newZ;
		clamp();
	}

	float getX() const { return x; }
	float getY() const { return y; }
	float getZ() const { return z; }

private:
	float x, y, z;

	void clamp() {
		x = constrainValue(x, -xRange / 2, xRange / 2);
		y = constrainValue(y, -yRange / 2, yRange / 2);
		z = constrainValue(z, 0, zRange);
	}

	float constrainValue(float value, float min, float max) {
		if (value < min) return min;
		if (value > max) return max;
		return value;
	}
};

// TODO: add a new Actuator object that initializes with motor type (lead or belt) and coreXY consideration
AccelStepper stepperR(motorInterfaceType, R, MOT_DIR_R);
AccelStepper stepperL(motorInterfaceType, MOT_STEP_L, MOT_DIR_L);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);
TMC2209Stepper driverR(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_R);
TMC2209Stepper driverL(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_L);
TMC2209Stepper driverZ(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_Z);
EncoderButton encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_BUTT);
Position desPos;

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

void disableStepperZ() {
	digitalWrite(MOT_EN_Z, HIGH);
}

void actuate(Position pos) {
	// actuate coreXY system
	float a = pos.getX() + pos.getY();
	float b = pos.getX() - pos.getY();
	stepperR.moveTo(a * ConvBelt);
	stepperL.moveTo(b * ConvBelt);
	stepperZ.moveTo(pos.getZ() * Conv);
}

void setup() {
	Serial.begin(115200);
	delay(100);

	// Set up systems
	motorSetup(); 
	driverSetup();

	stepperR.setMaxSpeed(maxSpeedAB);
	stepperL.setMaxSpeed(maxSpeedAB);
}

void loop() {
	if (Serial.available()) {
		char ch = Serial.read();
		switch (ch) {
			case '0':
				desPos.set(0.0f, 0.0f, 0.0f);
				break;
			case 'a':
				// move -x
				desPos.setX(desPos.getX() - 1.0f);
				break;
			case 'd':
				// move +x
				desPos.setX(desPos.getX() + 1.0f);
				break;
			case 's':
				// move -y
				desPos.setY(desPos.getY() - 1.0f);
				break;
			case 'w':
				// move +y
				desPos.setY(desPos.getY() + 1.0f);
				break;
		}
	}

	// Actuate system
	actuate(desPos);
	stepperR.run();
	stepperL.run();
	stepperZ.run();
}
