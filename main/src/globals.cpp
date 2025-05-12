#include "globals.h"
#include "config.h"

// Initialize hardware objects
AccelStepper stepperR(motorInterfaceType, MOT_STEP_R, MOT_DIR_R);
AccelStepper stepperL(motorInterfaceType, MOT_STEP_L, MOT_DIR_L);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);
TMC2209Stepper driverR(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_R);
TMC2209Stepper driverL(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_L);
TMC2209Stepper driverZ(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_Z);
Position desPos;
PMW3360 sensors[4];
EncoderButton encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_BUTT);
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, &SPI1);
Arduino_GFX *screen = new Arduino_GC9A01(bus, TFT_RST, 0, true);
SdFat sd;

// State variables
State state = POWER_ON;
CutState cutState = NOT_CUT_READY;
bool path_started = false;
bool valid_sensors = true;
DesignType designType = PRESET;
bool plungeReady = false;

// Path data
Path paths[MAX_PATHS];
int num_paths = 0;
int current_path_idx = 0;
int current_point_idx = 0;

// SD Stuff
FsFile logFile;
FsFile root;
FsFile currentDir;
int current_file_idx = 0;
int totalFiles = 0;
String fileList[MAX_FILES];

// Position tracking
RouterPose pose = {0.0f};
float measVel[2][4] = {{0.0f,0.0f,0.0f,0.0f},
						{0.0f,0.0f,0.0f,0.0f}};
float calPos[2][4] = {{0.0f,0.0f,0.0f,0.0f},
						{0.0f,0.0f,0.0f,0.0f}};

// Calibration data
CalParams cal[4];
float selfCal[2][3] = {{1.0f,0.997606f,0.989168f},
						{1.0f,1.004717f,1.000850f}};				// (UNUSED)

// Display variables
int16_t radius = 0;
int16_t centerX = 0;
int16_t centerY = 0;

// Mode select
bool plottingOn = false;			// plot values  (1 = yes; 0 = no)
bool debuggingOn = false;			// print values (1 = yes; 0 = no)
bool stopwatchOn = false;
bool outputOn = false;				// output data to serial
int designOrCalibrate = 0;			// choose design or calibrate (0 or 1)
int acceptCal = 0;					// accept calibration or not
int designPreset = 0;				// choose the design 

// Material properties
float matThickness = 0.0;

// Timing variables
long unsigned totalLoopTime = 0;
long unsigned sensingTime_debug = 0;
long unsigned stepperTime = 0;
long unsigned serialTime = 0;
long unsigned safetyTime = 0;
long unsigned cuttingTime = 0;
long unsigned timeLoopStart = 0;
long unsigned timeLastDebounce = 0;
long unsigned lastDraw = 0;
long unsigned timeLastPoll = 0;
long unsigned sensingTime = 0;
uint8_t iter = 0;