#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <PMW3360.h>
#include <SdFat.h>
#include <Arduino_GFX_Library.h>
#include <EncoderButton.h>

// Global object declarations
extern AccelStepper stepperX;
extern AccelStepper stepperZ;
extern TMC2209Stepper driverX;
extern TMC2209Stepper driverZ;
extern PMW3360 sensors[4];
extern EncoderButton encoder;
extern Arduino_GFX *screen;
extern SdFat sd;

// State variables
extern State state;
extern CutState cutState;
extern bool path_started;
extern bool valid_sensors;
extern float motorPosX;
extern DesignType designType;
extern bool plungeReady;

// Path data
extern Path paths[MAX_PATHS];
extern int num_paths;
extern int current_path_idx;
extern int current_point_idx;

// SD stuff
extern FsFile logFile;
extern FsFile root;
extern FsFile currentDir;
extern int current_file_idx;
extern int totalFiles;
extern String fileList[MAX_FILES];

// Position tracking
extern RouterPose pose;
extern float measVel[2][4];
extern float calPos[2][4];

// Calibration data
extern CalParams cal[4];

// Display variables
extern int16_t radius;
extern int16_t centerX;
extern int16_t centerY;

// Mode flags
extern bool plottingOn;
extern bool debuggingOn;
extern bool stopwatchOn;
extern bool outputOn;
extern int designOrCalibrate;
extern int acceptCal;
extern int designPreset;

// Material properties
extern float matThickness;

// Timing variables
extern long unsigned totalLoopTime;
extern long unsigned sensingTime_debug;
extern long unsigned stepperTime;
extern long unsigned serialTime;
extern long unsigned safetyTime;
extern long unsigned cuttingTime;
extern long unsigned timeLoopStart;
extern long unsigned timeLastDebounce;
extern long unsigned lastDraw;
extern long unsigned timeLastPoll;
extern long unsigned sensingTime;
extern uint8_t iter;

#endif