#include "GEM.h"

// --- 1. Define the Menu Object ---
GEM menu;

// --- 2. Define Menu Pages ---
GEMPage menuPageMain("Main Menu");
GEMPage menuPageSetThickness("Set Thickness");
GEMPage menuPageCutDesign("Cut Design");
GEMPage menuPagePreview("Preview Settings");
GEMPage menuPageCalibrate("Calibrate");
// ... etc.

// --- 3. Define Menu Items (linked to your variables) ---

// "Set Thickness" Page
// Links to your 'matThickness' float. (val, min, max, step)
GEMItem menuItemThickness("Thickness:", matThickness, 0.0, maxThickness, 0.1);

// "Preview Settings" Page
// Links directly to your boolean flags
GEMItem menuItemPreviewToggle("Preview:", enablePathPreview);
GEMItem menuItemPreviewMode("Mode:", pathPreviewFullScreen);

// "Main Menu" Page
// These items link to other pages
GEMItem menuItemGoToCut("Cut Design!", menuPageCutDesign);
GEMItem menuItemGoToCal("Calibrate", menuPageCalibrate);
GEMItem menuItemGoToPreview("Preview", menuPagePreview);

// "Cut Design" Page
// Example for your designType (PRESET=0, FROM_FILE=1, etc.)
// We cast your enum to a byte& for GEM
SelectOptionByte designOptions[] = {
    {"Preset", PRESET},
    {"From File", FROM_FILE},
    {"Speed Run", SPEED_RUN}
};
GEMSelect selectDesignType(sizeof(designOptions) / sizeof(SelectOptionByte), designOptions);
GEMItem menuItemDesignType("Type:", (byte&)designType, selectDesignType);