#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

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
#define ENCODER_BUTTON_PIN  4
#define MOT_EN_X    		23
#define MOT_DIR_X   		18
#define MOT_STEP_X  		19
#define MOT_EN_Z    		41
#define MOT_DIR_Z   		36
#define MOT_STEP_Z  		33
#define TFT_CS      		29
#define TFT_DC      		28
#define TFT_RST     		25
#define TFT_BL				24

// Driver settings
#define DRIVER_ADDRESS_X	0b00
#define DRIVER_ADDRESS_Z	0b10
#define R_SENSE            	0.11f
#define SERIAL_PORT     	Serial4

// Constants
#define MAX_PATHS  			10
#define MAX_POINTS 			1000
#define NUM_DESIGNS 		8
#define MAX_FILES			100
#define GC9A01A_WEBWORK_GREEN 0x8FF1
#define motorInterfaceType 	1
const float angleThreshold = PI/6;				// angle threshold for cuttable region (rads)

// Sensor properties
const int ns = 4;								// number of sensors
const int CPI = 2500;							// counts per inch. This value changes calibration coefficients
const float lx = 120.0f;						// x length of rectangular sensor configuration (mm)
const float ly = 140.0f;						// y length of rectangular sensor configuration (mm)

// Motor properties
const int uSteps = 4;							// microstep configuration
const float lead = 8;							// lead screw lead (mm)
const float Conv = (200*uSteps)/lead;			// conversion factor (mm -> steps)
const float stepPulseWidth = 20.0;				// min pulse width (from Mark Rober's code)
const float maxCurrent_RMS = 1273.0;			// motor RMS current rating (mA)
const float maxSpeedX = 140.0*Conv;				// max velocity X motor can move at (step/s)
const float maxSpeedZ = 180.0*Conv;				// max velocity Z motor can move at (step/s)
const float maxAccelX = 2000.0*Conv;				// x max acceleration (step/s^2)
const float maxAccelZ = 3000.0*Conv;			// z max acceleration (step/s^2)
const float retract = 5;						// distance to retract (mm)
const float zeroSpeed_0 = 20.0 * Conv;			// zeroing speed (step/s)
const float zeroSpeed_1 = 4.0 * Conv;			// secondary zeroing speed (step/s)
const float zeroAccel = 200.0 * Conv;			// zeroing acceleration (step/s^2)
const float holeFeedrate = 5.0*Conv; 			// default hole feedrate (step/s)		TODO: make this modifiable

// Material properties
const float maxThickness = 15.0;				// upper bound of thickness knob (mm)
const float restHeight = 2.0;					// rest height of tool before cutting (mm)
const float holeTolerance = 0.25;					// tolerance for hole positioning (mm)

// Gantry geometry
const float gantryLength = 100.0;				// usable length of x-gantry (mm)
const float gantryLengthRouter = gantryLength;	// usable length of x-gantry with router (mm) (default=gantryLength, DW611=94)
const float xLimitOffset = 2.54;				// distance from wall of stepper when zeroed (mm)
const float zLength = 34.0;						// usable length of z-gantry (mm)
const float zLimitOffset = 2.13;				// distance from wall when zeroed (mm)
const float wallBuffer = 3.0;						// safety buffer between tool body and walls (mm)
const float xSensorOffset = -2.32;				
const float ySensorOffset = -3.2;				// offset to try to counteract weird rotation behavior (mm) (UNUSED)

// Timing constants
const long unsigned debounceDelay = 50;			// the debounce time; increase if the output flickers
const long unsigned dtDebug = 500;				// (ms)
const long unsigned dtStopwatch = 500;				// time interval for loop timing (ms)
const long unsigned dtPlot = 50;				// (ms)
const long unsigned dtOutput = 20;				// (ms)
const long unsigned dtOutputSD = 10;			// (ms)
const long unsigned dt = 900;					// length of sensor timestep (us)(freq = 1,000,000/timestepPoll [Hz])

#endif
