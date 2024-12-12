#include "encoder.h"
#include "../config.h"
#include "../globals.h"
#include "../motors/motors.h"
#include "display.h"
#include "../path/path-generators.h"
#include "../io/logging.h"

void nullHandler(EncoderButton &eb) {
	Serial.println("null handler called");
	return;
}

void onClickGoToDesignMode(EncoderButton &eb) {
	encoderDesignMode();
}

void onClickGoToSetThickness(EncoderButton &eb) {
	encoderSetThickness();
}

void onClickResetState(EncoderButton &eb) {
	drawCenteredText("Zero Machine X", 1);
	state = POWER_ON;
	encoder.setClickHandler(onClickZeroMachineX);
}

void onClickZeroMachineX(EncoderButton &eb) {
	drawCenteredText("Zeroing Machine X...", 1);
	machineZeroX();
	state = MACHINE_X_ZERO;
	drawCenteredText("Zero Workspace Z", 1);
	encoder.setClickHandler(onClickZeroWorkspaceZ);
}

void onClickZeroWorkspaceZ(EncoderButton &eb) {
	drawCenteredText("Zeroing Workspace Z...", 1);
	workspaceZeroZ();
	state = WORKSPACE_Z_ZERO;
	encoderSetThickness();
}

void onClickZeroWorkspaceXY(EncoderButton &eb) {
	drawCenteredText("Zeroing Workspace XY...", 1);
	workspaceZeroXY();
	state = WORKSPACE_XY_ZERO;
}

void onEncoderUpdateThickness(EncoderButton &eb) {
	float incrScalar = 0.1;
	float tempThickness = matThickness + eb.increment()*incrScalar;

	if (tempThickness <= maxThickness && tempThickness >= 0.0) {
		matThickness = tempThickness;
	}
	
	char text2send[50];
	sprintf(text2send, "Turn to set thickness\n%.2f mm", matThickness);
	drawCenteredText(text2send, 1);
}

void onClickSetThickness(EncoderButton &eb) {
	state = THICKNESS_SET;
}

void onEncoderUpdateDesignMode(EncoderButton &eb) {
	designMode = (NUM_DESIGNS + designMode + eb.increment()) % NUM_DESIGNS;
	drawShape();
}

void onClickMakePath(EncoderButton &eb) {
	makePath();
	state = DESIGN_SELECTED;
}

void encoderSetThickness() {
	char text2send[50];
	sprintf(text2send, "Turn to set thickness\n%.2f mm", matThickness);
	drawCenteredText(text2send, 1);

	encoder.setEncoderHandler(onEncoderUpdateThickness);
	encoder.setClickHandler(onClickSetThickness);

	while (state != THICKNESS_SET) {
		encoder.update();
	}

	encoderDesignMode();
}

void encoderDesignMode() {
	drawShape();
	closeSDFile();

	state = SELECTING_DESIGN;
	encoder.setEncoderHandler(onEncoderUpdateDesignMode);
	encoder.setClickHandler(onClickMakePath);

	while (state != DESIGN_SELECTED) {
		encoder.update();
	}

	// Hack for opensauce, auto-zero XY
	workspaceZeroXY();

	// Reset cutting path
	path_started = false;
	current_path_idx = 0;
	current_point_idx = 0;

	state = READY;
	encoder.setEncoderHandler(nullHandler);
	encoder.setClickHandler(nullHandler);

	screen->fillScreen(BLACK);
	drawFixedUI();

	// Clear out sensors in case we moved while in design mode
	for (int i = 0; i < 4; i++) {
		sensors[i].readBurst();
	}
}

void encoderZeroWorkspaceXY() {
	drawCenteredText("Zero workspace XY", 1);
	encoder.setClickHandler(onClickZeroWorkspaceXY);

	while (state != WORKSPACE_XY_ZERO) {
		encoder.update();
	}

	if (!path_started) {
		// Reset cutting path
		path_started = false;
		current_path_idx = 0;
		current_point_idx = 0;
	}

	state = READY;
	encoder.setEncoderHandler(nullHandler);
	encoder.setClickHandler(nullHandler);

	screen->fillScreen(BLACK);
	drawFixedUI();
}
