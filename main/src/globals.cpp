#include "globals.h"
#include "config.h"

// Initialize hardware objects
AccelStepper stepperX(motorInterfaceType, MOT_STEP_X, MOT_DIR_X);
AccelStepper stepperZ(motorInterfaceType, MOT_STEP_Z, MOT_DIR_Z);
TMC2209Stepper driverX(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_X);
TMC2209Stepper driverZ(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS_Z);
PMW3360 sensors[4];
EncoderButton encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_BUTTON_PIN);
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

// Calibration data
float calPos[2][4] = {{0.0f,0.0f,0.0f,0.0f},
						{0.0f,0.0f,0.0f,0.0f}};		// used for calibration
CalParams cal[4];

// Display variables
int16_t radius = 0;
int16_t centerX = 0;
int16_t centerY = 0;

// Mode select
bool plottingOn = false;			// plot values  (1 = yes; 0 = no)
bool debuggingOn = false;			// print values (1 = yes; 0 = no)
bool stopwatchOn = false;
bool outputOn = false;				// output data to serial
bool outputSDOn = true;				// output data to SD card
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
long unsigned SDLogTime = 0;
long unsigned timeLoopStart = 0;
long unsigned timeLastDebounce = 0;
long unsigned lastDraw = 0;
long unsigned timeLastPoll = 0;
long unsigned sensingTime = 0;
uint8_t iter = 0;