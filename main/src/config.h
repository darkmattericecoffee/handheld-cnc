#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Pin definitions
#define SS0   39
#define SS1   9
#define SS2   32
#define SS3   40
#define LIMIT_MACH_X0       6
#define LIMIT_MACH_Z0       5
#define BUTT_HANDLE_L       2
#define BUTT_HANDLE_R       3
#define ENCODER_PIN_A       21
#define ENCODER_PIN_B       22
#define ENCODER_BUTTON_PIN  4
#define MOT_EN_X    18
#define MOT_DIR_X   14
#define MOT_STEP_X  15
#define MOT_EN_Z    41
#define MOT_DIR_Z   36
#define MOT_STEP_Z  33
#define TFT_CS      31
#define TFT_DC      30
#define TFT_RST     -1

// Driver settings
#define DRIVER_ADDRESS      0b00
#define R_SENSE            0.11f
#define SERIAL_PORT_X     Serial4
#define SERIAL_PORT_Z     Serial8

// Constants
#define MAX_PATHS  4
#define MAX_POINTS 1000
#define NUM_DESIGNS 9
#define GC9A01A_WEBWORK_GREEN 0x8FF1
#define motorInterfaceType 1

// Sensor properties
const int ns = 4;
const int CPI = 2500;
const float lx = 120.0f;
const float ly = 140.0f;

// Motor properties
const int uSteps = 4;
const float lead = 8;
const float Conv = 200/lead*uSteps;
const float stepPulseWidth = 20.0;
const float maxCurrent_RMS = 1273.0;
const float maxSpeedX = 200.0*Conv;
const float maxSpeedZ = 60.0*Conv;
const float maxAccel = 3000.0*Conv;
const float retract = 5;
const float speed_x0 = 20.0 * Conv;
const float speed_x1 = 4.0 * Conv;
const float accel_x0 = 200.0 * Conv;

// Material properties
const float maxThickness = 15.0;
const float restHeight = 2.0;

// Gantry geometry
const float gantryLength = 106.0;
const float xLimitOffset = 2.54;
const float xBuffer = 3.0;
const float xSensorOffset = -2.32;
const float ySensorOffset = -3.2;
const float zLength = 34.0;
const float zLimitOffset = 2.13;

// Timing constants
const long unsigned debounceDelay = 50;
const long unsigned dtDebug = 500;
const long unsigned dtPlot = 50;
const long unsigned dtOutput = 20;
const long unsigned dtOutputSD = 10;
const long unsigned dt = 900;

#endif
