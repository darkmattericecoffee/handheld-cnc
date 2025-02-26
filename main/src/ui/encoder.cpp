#include "encoder.h"
#include "../config.h"
#include "../globals.h"
#include "../motors/motors.h"
#include "display.h"
#include "../path/path-generators.h"
#include "../io/logging.h"
#include "../sensors/sensors.h"

void nullHandler(EncoderButton &eb) {
	Serial.println("null handler called");
	return;
}

// CLICK HANDLERS ----------------------------------------
void onClickGoToDesignMode(EncoderButton &eb) {
	encoderDesignType();
}

void onClickGoToSetThickness(EncoderButton &eb) {
	encoderSetThickness();
}

void onClickResetState(EncoderButton &eb) {
	drawCenteredText("Zero Machine X", 2);
	state = POWER_ON;
	encoder.setClickHandler(onClickZeroMachineX);
}

void onClickZeroMachineX(EncoderButton &eb) {
	drawCenteredText("Zeroing Machine X...", 2);
	machineZeroX();
	state = MACHINE_X_ZERO;
	drawCenteredText("Zero Workspace Z", 2);
	encoder.setClickHandler(onClickZeroWorkspaceZ);
}

void onClickZeroWorkspaceZ(EncoderButton &eb) {
	drawCenteredText("Zeroing Workspace Z...", 2);
	workspaceZeroZ();
	state = WORKSPACE_Z_ZERO;
	encoderSetThickness();
}

void onClickZeroWorkspaceXY(EncoderButton &eb) {
	drawCenteredText("Zeroing Workspace XY...", 1);
	workspaceZeroXY();
	state = WORKSPACE_XY_ZERO;
}

void onClickSetThickness(EncoderButton &eb) {
	state = THICKNESS_SET;
}

void onClickSetDoC(EncoderButton &eb) {
	state = DOC_SELECTED;
}

void onClickCalibrationAdvance(EncoderButton &eb) {
	state = CALIBRATION_ADVANCE;
}

void onClickSetType(EncoderButton &eb) {
	state = TYPE_SELECTED;
}

void onClickMakePath(EncoderButton &eb) {
	if (designType == PRESET) {
		makePresetPath();
		state = DESIGN_SELECTED;
	} else {
		handleFileSelection();
		if (state != DESIGN_SELECTED) {
			updateFileList();
			listFiles();
		}
	}
}

void onClickExecutePath(EncoderButton &eb) {
	if (paths[current_path_idx].feature == HOLE) {
		Serial.println("Plunge ready!");
		plungeReady = true;
	}
}

// ENCODER HANDLERS ----------------------------------------
void onEncoderUpdateThickness(EncoderButton &eb) {
	float incrScalar = 0.1;
	float tempThickness = matThickness + eb.increment()*incrScalar;

	if (tempThickness <= maxThickness && tempThickness >= 0.0) {
		matThickness = tempThickness;
	}
	
	char text2send[50];
	sprintf(text2send, "Turn to\nset thickness\n%.2f mm", matThickness);
	drawCenteredText(text2send, 2);
}

void onEncoderDesignOrCalibrate(EncoderButton &eb) {
	designOrCalibrate = (2 + designOrCalibrate + eb.increment()) % 2;
	drawDoCMenu();
}

// void onEncoderAcceptCalibration(EncoderButton &eb) {
// 	acceptCal = (2 + acceptCal + eb.increment()) % 2;
// 	drawDoCMenu();
// }

void onEncoderSwitchType(EncoderButton &eb) {
	designType = (DesignType)((2 + designType + eb.increment()) % 2);
	drawTypeMenu();
}

void onEncoderUpdateDesign(EncoderButton &eb) {
	if (designType == PRESET) {
		designPreset = (NUM_DESIGNS + designPreset + eb.increment()) % NUM_DESIGNS;
		drawShape();
	} else {
		current_file_idx = (totalFiles + current_file_idx + eb.increment()) % totalFiles;
		listFiles();
	}

}

// FUNCTIONS ------------------------------------------------
void encoderSetThickness() {
	char text2send[50];
	sprintf(text2send, "Turn to\nset thickness\n%.2f mm", matThickness);
	drawCenteredText(text2send, 2);

	encoder.setEncoderHandler(onEncoderUpdateThickness);
	encoder.setClickHandler(onClickSetThickness);

	while (state != THICKNESS_SET) {
		encoder.update();
	}

	encoderDesignOrCalibrate();
}

void encoderDesignOrCalibrate() {
	drawDoCMenu();

	encoder.setEncoderHandler(onEncoderDesignOrCalibrate);
	encoder.setClickHandler(onClickSetDoC);

	while (state != DOC_SELECTED) {
		encoder.update();
	}

	if (designOrCalibrate == 0) {
		encoderDesignType();
	} else {
		calibrate();
	}
}

void encoderDesignType() {
	drawTypeMenu();

	encoder.setEncoderHandler(onEncoderSwitchType);
	encoder.setClickHandler(onClickSetType);

	while (state != TYPE_SELECTED) {
		encoder.update();
	}
		
	encoderDesignSelect();
}

void encoderDesignSelect() {
	if (designType == PRESET) {
		drawShape();
	} else {
		updateFileList();
		listFiles();
	}
	
	closeSDFile();

	state = SELECTING_DESIGN;
	encoder.setEncoderHandler(onEncoderUpdateDesign);
	encoder.setClickHandler(onClickMakePath);

	while (state != DESIGN_SELECTED) {
		encoder.update();
		if (designType == FROM_FILE) listFiles();
	}

	// Hack for opensauce, auto-zero XY
	workspaceZeroXY();

	// Reset cutting path
	path_started = false;
	current_path_idx = 0;
	current_point_idx = 0;

	state = READY;
	encoder.setEncoderHandler(nullHandler);
	encoder.setClickHandler(onClickExecutePath);

	screen->fillScreen(BLACK);
	drawFixedUI();

	// Clear out sensors in case we moved while in design mode
	for (int i = 0; i < 4; i++) {
		sensors[i].readBurst();
	}
}

void encoderZeroWorkspaceXY() {
	drawCenteredText("Zero workspace XY", 2);
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
	encoder.setClickHandler(onClickExecutePath);

	screen->fillScreen(BLACK);
	drawFixedUI();
}
