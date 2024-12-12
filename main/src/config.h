#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Pin definitions
#define SS0   				39
// #define SS1   10
// #define SS2   40
// #define SS3   32
#define SS1   				9					// *** changed when soldering *** TODO: change back once ready
#define SS2   				32					// *** swapped when soldering *** TODO: swap back once ready
#define SS3					40					// *** swapped when soldering *** TODO: swap back once ready
#define LIMIT_MACH_X0       6
#define LIMIT_MACH_Z0       5
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
#define MAX_PATHS  			4
#define MAX_POINTS 			1000
#define NUM_DESIGNS 		9
#define GC9A01A_WEBWORK_GREEN 0x8FF1
#define motorInterfaceType 	1

// Sensor properties
const int ns = 4;								// number of sensors
const int CPI = 2500;							// counts per inch. This value changes calibration coefficients
const float lx = 120.0f;						// x length of rectangular sensor configuration (mm)
const float ly = 140.0f;						// y length of rectangular sensor configuration (mm)

// Motor properties
const int uSteps = 4;							// microstep configuration
const float lead = 8;							// lead screw lead (mm)
const float Conv = 200/lead*uSteps;				// conversion factor (mm -> steps)
const float stepPulseWidth = 20.0;				// min pulse width (from Mark Rober's code)
const float maxCurrent_RMS = 1273.0;			// motor RMS current rating (mA)
const float maxSpeedX = 200.0*Conv;				// max velocity X motor can move at (step/s)
const float maxSpeedZ = 60.0*Conv;				// max velocity Z motor can move at (step/s)
const float maxAccel = 3000.0*Conv;				// max acceleration (step/s^2)
const float retract = 5;						// distance to retract (mm)
const float speed_x0 = 20.0 * Conv;				// x zeroing speed (step/s)
const float speed_x1 = 4.0 * Conv;				// x secondary zeroing speed (step/s)
const float accel_x0 = 200.0 * Conv;			// x zeroing acceleration (step/s^2)

// Material properties
const float maxThickness = 15.0;				// upper bound of thickness knob (mm)
const float restHeight = 2.0;					// rest height of tool before cutting (mm)

// Gantry geometry
const float gantryLength = 106.0;				// usable length of x-gantry (mm)
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

#endif
