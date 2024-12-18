#include "globals.h"
#include "config.h"

// Initialize hardware objects
AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);
TMC2209Stepper driverX(&SERIAL_PORT_X, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverZ(&SERIAL_PORT_Z, R_SENSE, DRIVER_ADDRESS);
PMW3360 sensors[4];
EncoderButton encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_BUTTON_PIN);
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, &SPI1);
Arduino_GFX *screen = new Arduino_GC9A01(bus, TFT_RST, 0, true);
SdFat sd;
FsFile dataFile;

// State variables
State state = POWER_ON;
bool cutting = false;
bool path_started = false;
bool valid_sensors = true;

// Path data
Point paths[MAX_PATHS][MAX_POINTS];
int pathDir[MAX_PATHS] = {1,1,1,1,1,1,1,1,1,1};
int num_paths = 0;
int num_points = 0;
int current_path_idx = 0;
int current_point_idx = 0;

// Position tracking
float estPos[2] = {0.0f, 0.0f};
float estYaw = 0.0f;
float measVel[2][4] = {{0.0f,0.0f,0.0f,0.0f},
						{0.0f,0.0f,0.0f,0.0f}};

// Calibration data
float cVal[2][4] = {{0.0f,0.0f,0.0f,0.0f},
					{0.0f,0.0f,0.0f,0.0f}};
float selfCal[2][3] = {{1.0f,0.997606f,0.989168f},
						{1.0f,1.004717f,1.000850f}};

// Display variables
int16_t radius = 0;
int16_t centerX = 0;
int16_t centerY = 0;

// Mode flags
int plotting = 0;             // plot values  (1 = yes; 0 = no)
int debugMode = 0;            // print values (1 = yes; 0 = no)
int outputMode = 0;           // output data to serial
int designMode = 0;           // choose the design 

// Material properties
float matThickness = 0.0;

// Timing variables
long unsigned timeLastDebounce = 0;
long unsigned lastDraw = 0;
long unsigned timeLastPoll = 0;
long unsigned sensingTime = 0;
uint8_t iter = 0;