#ifndef ENCODER_CALLBACKS_H
#define ENCODER_CALLBACKS_H

#include <EncoderButton.h>

// Basic handlers
void nullHandler(EncoderButton &eb);
void onClickGoToDesignMode(EncoderButton &eb);
void onClickGoToSetThickness(EncoderButton &eb);
void onClickResetState(EncoderButton &eb);

// Zeroing handlers
void onClickZeroMachineX(EncoderButton &eb);
void onClickZeroWorkspaceZ(EncoderButton &eb);
void onClickZeroWorkspaceXY(EncoderButton &eb);

// Thickness handlers
void onEncoderUpdateThickness(EncoderButton &eb);
void onClickSetThickness(EncoderButton &eb);

// Design mode handlers
void onEncoderDesignOrCalibrate(EncoderButton &eb);
void onClickSetDoC(EncoderButton &eb);
void onEncoderUpdateDesign(EncoderButton &eb);
void onClickMakePath(EncoderButton &eb);

// Path execution handlers
void onClickExecutePath(EncoderButton &eb);

// Calibration handlers
void onClickCalibrationAdvance(EncoderButton &eb);
void onEncoderAcceptCalibration(EncoderButton &eb);
void onClickAcceptCalibration(EncoderButton &eb);

// Mode switching functions
void encoderSetThickness();
void encoderDesignOrCalibrate();
void encoderDesignType();
void encoderDesignSelect();
void encoderZeroWorkspaceXY();

#endif
