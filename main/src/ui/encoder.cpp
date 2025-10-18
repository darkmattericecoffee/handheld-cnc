#include "encoder.h"
#include "types.h"

// #define NUM_DESIGNS      9
#define NUM_DESIGNS         8

/*
 * NOTE ON EXTERNAL DEPENDENCIES:
 * This "module" relies on many global variables and functions defined in the 
 * main sketch (.ino) file, such as:
 * * - Globals: encoder, state, matThickness, mainMenuSelection, previewMenuSelection,
 * enablePathPreview, pathPreviewFullScreen, designType, designPreset,
 * feedrate, cutState, screen, etc.
 * - Enums:   StateType (POWER_ON, ZEROED, THICKNESS_SET, etc.)
 * DesignType (PRESET, FROM_FILE, etc.)
 * - Functions: drawCenteredText(), drawMenu(), machineZeroXY(), workspaceZeroZ(),
 * workspaceZeroXY(), makePresetPath(), updateFileList(), listFiles(),
 * calibrate(), drawShape(), drawFixedUI(), etc.
 */


// --- 0. General & Helper Functions ---

/**
 * @brief A null handler that does nothing. Useful for disabling a 
 * click/rotate action in certain states.
 */
static void nullHandler(EncoderButton &eb) {
    Serial.println("null handler called");
    return;
}

/**
 * @brief Helper function to draw the preview settings menu.
 * Avoids duplicating this drawing logic in multiple handlers.
 */
static void drawPreviewMenu() {
    screen->fillScreen(BLACK);
    screen->setTextSize(2);
    
    // Calculate vertical spacing
    int16_t yStart = screen->height() / 3;
    int16_t ySpacing = 30;
    
    // Define menu items
    const char* labels[] = {"Preview:", "Mode:", "Back"};
    const char* values[] = {
        enablePathPreview ? "ON" : "OFF",
        pathPreviewFullScreen ? "Full" : "Rect",
        ""
    };
    
    // Draw each option
    for (int i = 0; i < 3; i++) {
        // Set color based on selection
        uint16_t textColor = (i == previewMenuSelection) ? YELLOW : WHITE;
        screen->setTextColor(textColor);
        
        int16_t y = yStart + (i * ySpacing);
        
        if (i < 2) {  // Options with indicators
            // Draw label
            screen->setCursor(30, y);
            screen->print(labels[i]);
            
            // Draw value
            screen->setCursor(30 + strlen(labels[i]) * 12 + 5, y);
            screen->print(values[i]);
            
            // Draw indicator circle (filled if enabled, empty if disabled)
            bool showFilled = (i == 0) ? enablePathPreview : pathPreviewFullScreen;
            int16_t circleX = screen->width() - 30;
            int16_t circleY = y + 8;  // Center vertically with text
            drawIndicator(circleX, circleY, showFilled, textColor);
        } else {  // Back option
            // Center the "Back" text
            int16_t x1, y1;
            uint16_t w, h;
            screen->getTextBounds("< Back", 0, 0, &x1, &y1, &w, &h);
            int16_t x = (screen->width() - w) / 2;
            screen->setCursor(x, y);
            screen->print("< Back");
        }
    }
}

/**
 * @brief Runs a blocking encoder update loop until the global 'state'
 * matches the targetState or becomes READY.
 * @param targetState The state to wait for, set by a click handler.
 */
static void runMenuLoop(State targetState) {
    while (state != targetState && state != READY) {
        encoder.update();
    }
}


// --- 1. Initial Zeroing (Chained Handlers) ---

static void onClickZeroWorkspaceZ(EncoderButton &eb) {
    drawCenteredText("Zeroing Workspace Z...", 2);
    workspaceZeroZ();
    state = WORKSPACE_Z_ZERO;
    encoderSetThickness();
}

static void onClickZeroMachineXY(EncoderButton &eb) {
    drawCenteredText("Zeroing Machine XY...", 2);
    machineZeroXY();
    state = MACHINE_XY_ZERO;
    drawCenteredText("Zero Workspace Z", 2);
    encoder.setClickHandler(onClickZeroWorkspaceZ);
}


// --- 2. Workspace XY Zeroing (Menu Function) ---

static void onClickZeroWorkspaceXY(EncoderButton &eb) {
    drawCenteredText("Zeroing Workspace XY...", 1);
    workspaceZeroXY();
    state = WORKSPACE_XY_ZERO;
}

/**
 * @brief PUBLIC: Displays the "Zero workspace XY" screen and waits for a click.
 */
void encoderZeroWorkspaceXY() {
    drawCenteredText("Zero workspace XY", 2);
    encoder.setClickHandler(onClickZeroWorkspaceXY);

    // Use the blocking loop helper
    runMenuLoop(WORKSPACE_XY_ZERO);

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


// --- 3. Set Thickness ---

static void onClickSetThickness(EncoderButton &eb) {
    state = THICKNESS_SET;
}

static void onEncoderUpdateThickness(EncoderButton &eb) {
    float incrScalar = 0.1;
    float tempThickness = matThickness + eb.increment()*incrScalar;

    if (tempThickness <= maxThickness && tempThickness >= 0.0) {
        matThickness = tempThickness;
    }
    
    char text2send[50];
    sprintf(text2send, "Turn to\nset thickness\n%.2f mm", matThickness);
    drawCenteredText(text2send, 2);
}

/**
 * @brief PUBLIC: Enters the "Set Thickness" menu loop.
 */
void encoderSetThickness() {
    char text2send[50];
    sprintf(text2send, "Turn to\nset thickness\n%.2f mm", matThickness);
    drawCenteredText(text2send, 2);

    encoder.setEncoderHandler(onEncoderUpdateThickness);
    encoder.setClickHandler(onClickSetThickness);

    // Use the blocking loop helper
    runMenuLoop(THICKNESS_SET);

    encoderDesignOrCalibrate();
}


// --- 4. Main Menu (Design/Calibrate/Preview) ---

static void onClickSetDoC(EncoderButton &eb) {
    state = DOC_SELECTED;
}

static void onEncoderMainMenu(EncoderButton &eb) {
    mainMenuSelection = (3 + mainMenuSelection + eb.increment()) % 3;
    const char* options[] = {"Cut Design!", "Calibrate", "Preview Settings"};
    drawMenu(options, 3, mainMenuSelection);
}

/**
 * @brief PUBLIC: Enters the "Design or Calibrate" (main) menu loop.
 */
void encoderDesignOrCalibrate() {
    const char* options[] = {"Cut Design!", "Calibrate", "Preview Settings"};
    drawMenu(options, 3, mainMenuSelection);

    encoder.setEncoderHandler(onEncoderMainMenu);
    encoder.setClickHandler(onClickSetDoC);

    // Use the blocking loop helper
    runMenuLoop(DOC_SELECTED);

    if (mainMenuSelection == 0) {
        encoderDesignType();
    } else if (mainMenuSelection == 1) {
        calibrate();
    } else {
        encoderPreviewSettings();
        // After preview settings, return to this menu
        state = THICKNESS_SET;  // Reset state to re-enter the main menu loop
        encoderDesignOrCalibrate();
    }
}


// --- 5. Preview Settings Menu ---

static void onClickPreviewSettings(EncoderButton &eb) {
    Serial.printf("Preview click: selection=%d\n", previewMenuSelection);
    if (previewMenuSelection == 0) {
        // Toggle preview on/off
        enablePathPreview = !enablePathPreview;
        Serial.printf("Toggled preview to: %s\n", enablePathPreview ? "ON" : "OFF");
    } else if (previewMenuSelection == 1) {
        // Toggle full screen mode (only if preview is enabled)
        if (enablePathPreview) {
            pathPreviewFullScreen = !pathPreviewFullScreen;
            Serial.printf("Toggled full screen to: %s\n", pathPreviewFullScreen ? "ON" : "OFF");
        }
    } else {
        // Back to main menu - use a special state to signal exit
        Serial.println("Exiting preview settings");
        state = THICKNESS_SET;  // Signal to exit preview settings menu
        return; // Don't redraw if we're exiting
    }
    
    // Immediately update visuals by calling the helper
    drawPreviewMenu();
}

static void onEncoderPreviewSettings(EncoderButton &eb) {
    int oldSelection = previewMenuSelection;
    previewMenuSelection = (3 + previewMenuSelection + eb.increment()) % 3;
    
    if (oldSelection != previewMenuSelection) {
        Serial.printf("Preview selection changed from %d to %d\n", oldSelection, previewMenuSelection);
    }
    
    // Update visuals by calling the helper
    drawPreviewMenu();
}

/**
 * @brief PUBLIC: Enters the "Preview Settings" menu loop.
 */
void encoderPreviewSettings() {
    Serial.println("Entering preview settings menu");
    previewMenuSelection = 0;
    bool inPreviewMenu = true;
    
    // Clear encoder state and reset position
    encoder.update();
    encoder.resetPosition(); 
    
    encoder.setEncoderHandler(onEncoderPreviewSettings);
    encoder.setClickHandler(onClickPreviewSettings);

    // Initial draw
    drawPreviewMenu();
    Serial.printf("Initial preview selection: %d\n", previewMenuSelection);

    while (inPreviewMenu && state != READY) {
        encoder.update(); // This will trigger handlers which handle redraws
        
        // Check if user wants to exit preview settings (state set in onClick)
        if (state == THICKNESS_SET) {
            Serial.println("Exiting preview settings menu");
            inPreviewMenu = false;
        }
        
        // The complex redraw logic with 'lastClickCount' is no longer needed
        // because the click and encoder handlers now call drawPreviewMenu() directly.
    }
    
    // Reset mainMenuSelection so we return to the main menu properly
    mainMenuSelection = 0;
    
    Serial.println("Preview settings function completed");
}


// --- 6. Design Type Select ---

static void onClickSetType(EncoderButton &eb) {
    state = TYPE_SELECTED;
}

static void onEncoderSwitchType(EncoderButton &eb) {
    designType = (DesignType)((3 + designType + eb.increment()) % 3);
    const char* options[] = {"Preset", "From File", "Speed Run"};
    drawMenu(options, 3, designType);
}

/**
 * @brief PUBLIC: Enters the "Design Type" (Preset/File/Speed) menu loop.
 */
void encoderDesignType() {
    const char* options[] = {"Preset", "From File", "Speed Run"};
    drawMenu(options, 3, designType);

    encoder.setEncoderHandler(onEncoderSwitchType);
    encoder.setClickHandler(onClickSetType);

    // Use the blocking loop helper
    runMenuLoop(TYPE_SELECTED);
        
    encoderDesignSelect();
}


// --- 7. Design/File/Speed Select ---

static void onClickMakePath(EncoderButton &eb) {
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

static void onEncoderUpdateDesign(EncoderButton &eb) {
    if (designType == PRESET) {
        designPreset = (NUM_DESIGNS + designPreset + eb.increment()) % NUM_DESIGNS;
        drawShape();
    } else {
        current_file_idx = (totalFiles + current_file_idx + eb.increment()) % totalFiles;
        Serial.printf("File index: %i\n", current_file_idx);
        listFiles();
    }
}

static void onEncoderSetSpeed(EncoderButton &eb) {
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

static void onEncoderSetBoost(EncoderButton &eb) {
    float incrScalar = 0.1;
    float tempBoost = feedrateBoost + eb.increment()*incrScalar;

    if (tempBoost >= 0.1 && tempBoost <= 5.0) {
        feedrateBoost = tempBoost;
    }

    Serial.printf("Feedrate boost set to: %.2f\n", feedrateBoost);
}

/**
 * @brief PUBLIC: Enters the "Select Design/File/Speed" menu loop.
 */
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

    // Use a custom loop here as it has extra logic
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
    if (designType != SPEED_RUN) feedrate = feedrate_default;
    feedrateBoost = 1.0;
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


// --- 8. Pause Menu ---

static void onClickPauseCut(EncoderButton &eb) {
    if (cutState == CUTTING) {
        cutState = NOT_USER_READY;
    } else if (cutState == NOT_USER_READY) {
        cutState = CUT_READY;
    }

    state = STANDBY;
    encoderHandlePause();
}

static void onClickPauseSelect(EncoderButton &eb) {
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

static void onEncoderPauseMenu(EncoderButton &eb) {
    designOrCalibrate = (2 + designOrCalibrate + eb.increment()) % 2;
    const char* options[] = {"Re-zero XY", "Main Menu"};
    drawMenu(options, 2, designOrCalibrate);
}

/**
 * @brief PUBLIC: Enters the "Pause" menu loop.
 */
void encoderHandlePause() {
    const char* options[] = {"Re-zero XY", "Main Menu"};
    drawMenu(options, 2, designOrCalibrate);

    encoder.setEncoderHandler(onEncoderPauseMenu);
    encoder.setClickHandler(onClickPauseSelect);

    // Note: The original code waited for DOC_SELECTED, but the click
    // handler sets WORKSPACE_XY_ZERO or ZEROED. This loop seems intended
    // to wait until the state *changes* from STANDBY.
    // We'll wait for a state that isn't the one we started in.
    while (state == STANDBY) {
        encoder.update();
    }
    // After the click handler, state will be WORKSPACE_XY_ZERO or ZEROED,
    // and the corresponding function will have been called.
}


// --- 9. Calibration Handlers ---
// (These are just handlers, called by the external 'calibrate()' function)

static void onClickCalibrationAdvance(EncoderButton &eb) {
    state = CALIBRATION_ADVANCE;
}

static void onClickAcceptCalibration(EncoderButton &eb) {
    state = CALIBRATION_ADVANCE;
}

static void onEncoderAcceptCalibration(EncoderButton &eb) {
    acceptCal = (2 + acceptCal + eb.increment()) % 2;
    const char* options[] = {"Exit", "Save!"};
    drawMenu(options, 2, acceptCal);
}


// --- 10. End Screen ---

static void onClickEndScreen(EncoderButton &eb) {
    state = POWER_ON;
}

/**
 * @brief PUBLIC: Displays the "End Screen" and waits for a click to reset.
 */
void encoderEndScreen() {
    if (designType != SPEED_RUN) {
        drawCenteredText("WOOO nice job!", 2);
    } else {
        char text2send[50];
        sprintf(text2send, "WOW!\nYour time was:\n%.2fs", speedRunTimer/1000.0);
        drawCenteredText(text2send, 2);
        feedrate = feedrate_default;
    }

    encoder.setClickHandler(onClickEndScreen);
    
    // Use the blocking loop helper
    runMenuLoop(POWER_ON);
}


// --- 11. Other Handlers ---

// Reset design (from triple click)
static void onClickResetState(EncoderButton &eb) {
    state = ZEROED;
    // TODO: reset goal point after triple click

    encoderSetThickness();
}