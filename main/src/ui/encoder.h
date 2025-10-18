#ifndef ENCODER_CALLBACKS_H
#define ENCODER_CALLBACKS_H

#include <EncoderButton.h>
#include "../config.h"
#include "../globals.h"
#include "../actuation/motors.h"
#include "display.h"
#include "../path/path-generators.h"
#include "../io/logging.h"
#include "../sensors/sensors.h"

// Basic handlers
void nullHandler(EncoderButton &eb);
void onClickGoToSetThickness(EncoderButton &eb);
void onClickResetState(EncoderButton &eb);

// Zeroing handlers
void onClickZeroMachineXY(EncoderButton &eb);
void onClickZeroWorkspaceZ(EncoderButton &eb);
void onClickZeroWorkspaceXY(EncoderButton &eb);

// Thickness handlers
void onEncoderUpdateThickness(EncoderButton &eb);
void onClickSetThickness(EncoderButton &eb);

// Design mode handlers
void onEncoderMainMenu(EncoderButton &eb);
void onClickSetDoC(EncoderButton &eb);
void onEncoderUpdateDesign(EncoderButton &eb);
void onClickMakePath(EncoderButton &eb);

// Preview settings handlers
void onEncoderPreviewSettings(EncoderButton &eb);
void onClickPreviewSettings(EncoderButton &eb);

// Path execution handlers
void onEncoderSetSpeed(EncoderButton &eb);
void onEncoderSetBoost(EncoderButton &eb);
void onClickSetSpeed(EncoderButton &eb);
void onClickPauseCut(EncoderButton &eb);
void onEncoderPauseMenu(EncoderButton &eb);
void onClickPauseSelect(EncoderButton &eb);

// Calibration handlers
void onClickCalibrationAdvance(EncoderButton &eb);
void onEncoderAcceptCalibration(EncoderButton &eb);
void onClickAcceptCalibration(EncoderButton &eb);

// Mode switching functions
void encoderSetThickness();
void encoderDesignOrCalibrate();
void encoderPreviewSettings();
void encoderDesignType();
void encoderDesignSelect();
void encoderZeroWorkspaceXY();
void encoderHandlePause();
void encoderEndScreen();

#endif
