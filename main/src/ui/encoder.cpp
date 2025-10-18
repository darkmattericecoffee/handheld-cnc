#include "encoder.h"

// #define NUM_DESIGNS 		9
#define NUM_DESIGNS 		8


void nullHandler(EncoderButton &eb) {
	Serial.println("null handler called");
	return;
}

// CLICK HANDLERS ----------------------------------------
void onClickGoToDesignMode(EncoderButton &eb) {
	encoderDesignType();
}

// Reset design (from triple click)
void onClickResetState(EncoderButton &eb) {
	state = ZEROED;
	// TODO: reset goal point after triple click

	encoderSetThickness();
}

void onClickZeroMachineXY(EncoderButton &eb) {
	drawCenteredText("Zeroing Machine XY...", 2);
	machineZeroXY();
	state = MACHINE_XY_ZERO;
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

void onClickPreviewSettings(EncoderButton &eb) {
	Serial.printf("Preview click: selection=%d\n", previewMenuSelection);
	if (previewMenuSelection == 0) {
		// Toggle preview on/off
		enablePathPreview = !enablePathPreview;
		Serial.printf("Toggled preview to: %s\n", enablePathPreview ? "ON" : "OFF");
		// Immediately update visuals - force full menu redraw
		char option0[30], option1[30], option2[30];
		sprintf(option0, "Preview: %s", enablePathPreview ? "ON ●" : "OFF ○");
		sprintf(option1, "Mode: %s", pathPreviewFullScreen ? "Full ●" : "Rect ○");  
		strcpy(option2, "← Back");
		const char* options[] = {option0, option1, option2};
		drawMenu(options, 3, previewMenuSelection);
	} else if (previewMenuSelection == 1) {
		// Toggle full screen mode (only if preview is enabled)
		if (enablePathPreview) {
			pathPreviewFullScreen = !pathPreviewFullScreen;
			Serial.printf("Toggled full screen to: %s\n", pathPreviewFullScreen ? "ON" : "OFF");
			// Immediately update visuals - force full menu redraw
			char option0[30], option1[30], option2[30];
			sprintf(option0, "Preview: %s", enablePathPreview ? "ON ●" : "OFF ○");
			sprintf(option1, "Mode: %s", pathPreviewFullScreen ? "Full ●" : "Rect ○");  
			strcpy(option2, "← Back");
			const char* options[] = {option0, option1, option2};
			drawMenu(options, 3, previewMenuSelection);
		}
	} else {
		// Back to main menu - use a special state to signal exit
		Serial.println("Exiting preview settings");
		state = THICKNESS_SET;  // Signal to exit preview settings menu
	}
}

void onClickCalibrationAdvance(EncoderButton &eb) {
	state = CALIBRATION_ADVANCE;
}

void onClickAcceptCalibration(EncoderButton &eb) {
	state = CALIBRATION_ADVANCE;
}

void onClickSetType(EncoderButton &eb) {
	state = TYPE_SELECTED;
}

void onClickMakePath(EncoderButton &eb) {
	if (designType == PRESET) {
		makePresetPath();
		state = DESIGN_SELECTED;
	} else if (designType == FROM_FILE) {
		handleFileSelection();
		if (state != DESIGN_SELECTED) {
			updateFileList();
			listFiles();
		}
	} else {
		handleSpeedRun();
		state = DESIGN_SELECTED;
	}
}

void onClickPauseCut(EncoderButton &eb) {
	if (cutState == CUTTING) {
		cutState = NOT_USER_READY;
	} else if (cutState == NOT_USER_READY) {
		cutState = CUT_READY;
	}

	state = STANDBY;
	encoderHandlePause();
}

void onClickPauseSelect(EncoderButton &eb) {
	if (pauseSelection == 0) {
		// Re-zero XY
		workspaceZeroXY();
		state = WORKSPACE_XY_ZERO;
	} else {
		// Go to main menu
		state = ZEROED;
		encoderSetThickness();
	}
}

void onClickEndScreen(EncoderButton &eb) {
	state = POWER_ON;
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

void onEncoderMainMenu(EncoderButton &eb) {
	mainMenuSelection = (3 + mainMenuSelection + eb.increment()) % 3;
	const char* options[] = {"Cut Design!", "Calibrate", "Preview Settings"};
	drawMenu(options, 3, mainMenuSelection);
}

void onEncoderPreviewSettings(EncoderButton &eb) {
	int oldSelection = previewMenuSelection;
	previewMenuSelection = (3 + previewMenuSelection + eb.increment()) % 3;
	
	if (oldSelection != previewMenuSelection) {
		Serial.printf("Preview selection changed from %d to %d\n", oldSelection, previewMenuSelection);
	}
	
	// Create visual menu with icons/indicators
	char option0[30], option1[30], option2[30];
	sprintf(option0, "Preview: %s", enablePathPreview ? "ON ●" : "OFF ○");
	sprintf(option1, "Mode: %s", pathPreviewFullScreen ? "Full ●" : "Rect ○");  
	strcpy(option2, "← Back");
	
	const char* options[] = {option0, option1, option2};
	drawMenu(options, 3, previewMenuSelection);
}

void onEncoderAcceptCalibration(EncoderButton &eb) {
	acceptCal = (2 + acceptCal + eb.increment()) % 2;
	const char* options[] = {"Exit", "Save!"};
	drawMenu(options, 2, acceptCal);
}

void onEncoderSwitchType(EncoderButton &eb) {
	designType = (DesignType)((3 + designType + eb.increment()) % 3);
	const char* options[] = {"Preset", "From File", "Speed Run"};
	drawMenu(options, 3, designType);
}

void onEncoderUpdateDesign(EncoderButton &eb) {
	if (designType == PRESET) {
		designPreset = (NUM_DESIGNS + designPreset + eb.increment()) % NUM_DESIGNS;
		drawShape();
	} else {
		current_file_idx = (totalFiles + current_file_idx + eb.increment()) % totalFiles;
		Serial.printf("File index: %i\n", current_file_idx);
		listFiles();
	}

}

void onEncoderPauseMenu(EncoderButton &eb) {
	designOrCalibrate = (2 + designOrCalibrate + eb.increment()) % 2;
	const char* options[] = {"Re-zero XY", "Main Menu"};
	drawMenu(options, 2, designOrCalibrate);
}

void onEncoderSetSpeed(EncoderButton &eb) {
	float maxSpeed = 0.5 * maxSpeedAB / ConvBelt;
	float incrScalar = 1.0;
	float tempSpeed = feedrate + eb.increment()*incrScalar;

	if (tempSpeed <= maxSpeed && tempSpeed >= 1.0) {
		feedrate = tempSpeed;
	}
	
	if (state != READY) {
		char text2send[50];
		sprintf(text2send, "Turn to\nset speed\n%.2f mm/s", feedrate);
		drawCenteredText(text2send, 2);
	} else {
		Serial.printf("Feedrate set to: %.2f mm/s\n", feedrate);
	}
}

void onEncoderSetBoost(EncoderButton &eb) {
	float incrScalar = 0.1;
	float tempBoost = feedrateBoost + eb.increment()*incrScalar;

	if (tempBoost >= 0.1 && tempBoost <= 5.0) {
		feedrateBoost = tempBoost;
	}

	Serial.printf("Feedrate boost set to: %.2f\n", feedrateBoost);
}

// FUNCTIONS ------------------------------------------------
void encoderSetThickness() {
	char text2send[50];
	sprintf(text2send, "Turn to\nset thickness\n%.2f mm", matThickness);
	drawCenteredText(text2send, 2);

	encoder.setEncoderHandler(onEncoderUpdateThickness);
	encoder.setClickHandler(onClickSetThickness);

	while (state != THICKNESS_SET  && state != READY) {
		encoder.update();
	}

	encoderDesignOrCalibrate();
}

void encoderDesignOrCalibrate() {
	const char* options[] = {"Cut Design!", "Calibrate", "Preview Settings"};
	drawMenu(options, 3, mainMenuSelection);

	encoder.setEncoderHandler(onEncoderMainMenu);
	encoder.setClickHandler(onClickSetDoC);

	while (state != DOC_SELECTED  && state != READY) {
		encoder.update();
	}

	if (mainMenuSelection == 0) {
		encoderDesignType();
	} else if (mainMenuSelection == 1) {
		calibrate();
	} else {
		encoderPreviewSettings();
		// After preview settings, return to this menu (state is still THICKNESS_SET)
		state = THICKNESS_SET;  // Reset state to re-enter the main menu loop
		encoderDesignOrCalibrate();
	}
}

void encoderPreviewSettings() {
	Serial.println("Entering preview settings menu");
	// Set initial preview menu display
	previewMenuSelection = 0;
	bool inPreviewMenu = true;
	
	// Important: Clear encoder state and reset position
	encoder.update(); // Clear any pending events
	encoder.resetPosition(); // Reset encoder position to avoid carryover
	
	encoder.setEncoderHandler(onEncoderPreviewSettings);
	encoder.setClickHandler(onClickPreviewSettings);

	// Initial draw with explicit call (no increment)
	char option0[30], option1[30], option2[30];
	sprintf(option0, "Preview: %s", enablePathPreview ? "ON ●" : "OFF ○");
	sprintf(option1, "Mode: %s", pathPreviewFullScreen ? "Full ●" : "Rect ○");  
	strcpy(option2, "← Back");
	const char* options[] = {option0, option1, option2};
	drawMenu(options, 3, previewMenuSelection);
	
	Serial.printf("Initial preview selection: %d\n", previewMenuSelection);

	while (inPreviewMenu && state != READY) {
		encoder.update();
		
		// Check if user wants to exit preview settings
		if (state == THICKNESS_SET) {
			Serial.println("Exiting preview settings menu");
			inPreviewMenu = false;
		}
		
		// Redraw menu to update visual indicators when settings change
		static int lastClickCount = 0;
		if (lastClickCount == 0) {
			// initialize sentinel to current click count on entry
			lastClickCount = encoder.clickCount();
		}
		if (encoder.clickCount() != lastClickCount && previewMenuSelection < 2) {
			lastClickCount = encoder.clickCount();
			onEncoderPreviewSettings(encoder);
		}
	}
	
	// Reset mainMenuSelection so we return to the main menu properly
	mainMenuSelection = 0;
	// Don't change state - let encoderDesignOrCalibrate() handle it
	
	Serial.println("Preview settings function completed");
}

void encoderDesignType() {
	const char* options[] = {"Preset", "From File", "Speed Run"};
	drawMenu(options, 3, designType);

	encoder.setEncoderHandler(onEncoderSwitchType);
	encoder.setClickHandler(onClickSetType);

	while (state != TYPE_SELECTED  && state != READY) {
		encoder.update();
	}
		
	encoderDesignSelect();
}

void encoderDesignSelect() {
	state = SELECTING_DESIGN;
	encoder.setEncoderHandler(onEncoderUpdateDesign);
	encoder.setClickHandler(onClickMakePath);

	if (designType == PRESET) {
		drawShape();
	} else if (designType == FROM_FILE){
		updateFileList();
		listFiles();
	} else {
		char text2send[50];
		sprintf(text2send, "Turn to\nset speed\n%.2f mm/s", feedrate);
		drawCenteredText(text2send, 2);

		encoder.setEncoderHandler(onEncoderSetSpeed);
	}
	
	closeSDFile();

	while (state != DESIGN_SELECTED) {
		encoder.update();
		if (designType == FROM_FILE) listFiles();
	}

	// Reset pose
	workspaceZeroXY();
	distanceTraveled = 0.0f;

	// Reset cutting path
	running = true;
	current_point_idx = 0;
	if (designType != SPEED_RUN) feedrate = feedrate_default;	// reset feedrate to default (NOTE: only RMRRF addition)
	feedrateBoost = 1.0;	// reset feedrate boost to default
	speedRunTimer = 0;

	state = READY;
	cutState = NOT_CUT_READY;
	encoder.setEncoderHandler(onEncoderSetBoost);
	encoder.setClickHandler(nullHandler);

	screen->fillScreen(BLACK);
	drawFixedUI();

	// Clear out sensors in case we moved while in design mode
	for (int i = 0; i < 4; i++) {
		sensors[i].readBurst();
	}
}

void encoderHandlePause() {
	const char* options[] = {"Re-zero XY", "Main Menu"};
	drawMenu(options, 2, designOrCalibrate);

	encoder.setEncoderHandler(onEncoderPauseMenu);
	encoder.setClickHandler(onClickPauseSelect);

	while (state != DOC_SELECTED) {
		encoder.update();
	}
}

void encoderZeroWorkspaceXY() {
	drawCenteredText("Zero workspace XY", 2);
	encoder.setClickHandler(onClickZeroWorkspaceXY);

	while (state != WORKSPACE_XY_ZERO  && state != READY) {
		encoder.update();
	}

	if (!running) {
		// TODO: look into this. Not really sure the use case here. Maybe re-zeroing mid cut?
		// Reset cutting path
		running = true;
		current_point_idx = 0;
	}

	state = READY;
	resetPathPreview();  // Reset path preview when entering READY state
	encoder.setEncoderHandler(nullHandler);
	encoder.setClickHandler(nullHandler);

	screen->fillScreen(BLACK);
	drawFixedUI();
}

void encoderEndScreen() {
	if (designType != SPEED_RUN) {
		drawCenteredText("WOOO nice job!", 2);
	} else {
		char text2send[50];
		sprintf(text2send, "WOW!\nYour time was:\n%.2fs", speedRunTimer/1000.0);
		drawCenteredText(text2send, 2);
		feedrate = feedrate_default;	// reset feedrate to default (NOTE: only RMRRF addition)
	}

	// stay here until encoder is clicked
	encoder.setClickHandler(onClickEndScreen); // prevent accidental state changes
	while (state != POWER_ON) {
		encoder.update();
	}
}
