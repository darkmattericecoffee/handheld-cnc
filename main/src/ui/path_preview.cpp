#include "path_preview.h"
#include "globals.h" // Access to 'screen' and 'path'
#include "config.h"  // Access to 'MAX_POINTS', 'GC9A01A_WEBWORK_GREEN', etc.
#include "types.h"   // Access to 'restHeight' and 'Feature' enum
#include "../math/geometry.h" // Access to 'mapF' function

// Structure to store a line segment for clearing
struct PathLine {
    int16_t x1, y1, x2, y2;
    uint16_t color;
    bool valid;
};

// Structure to store a circle for clearing
struct PathCircle {
    int16_t x, y, r;
    uint16_t color;
    bool valid;
};

// Static storage for previously drawn elements (limit to reasonable number)
static const int MAX_STORED_LINES = 200;
static const int MAX_STORED_CIRCLES = 50;
static PathLine previousLines[MAX_STORED_LINES];
static PathCircle previousCircles[MAX_STORED_CIRCLES];
static int numPreviousLines = 0;
static int numPreviousCircles = 0;

void clearPreviousPathPreview() {
    // Fast clear: just fill the UI rectangle area with black
    // This is much faster than clearing individual lines
    float padding = 6;
    float rectangleWidth = screen->width() / 2;
    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;
    
    screen->fillRect(
        centerX - rectangleWidth/2 + padding,
        centerY - rectangleWidth/2 + padding,
        rectangleWidth - 2*padding,
        rectangleWidth - 2*padding,
        BLACK
    );
    
    numPreviousLines = 0;  // Reset the counts (though we don't use them anymore)
    numPreviousCircles = 0;
}

void storeLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    // Store line for later clearing, but don't exceed our storage limit
    if (numPreviousLines < MAX_STORED_LINES) {
        previousLines[numPreviousLines] = {x1, y1, x2, y2, color, true};
        numPreviousLines++;
    }
}

void storeCircle(int16_t x, int16_t y, int16_t r, uint16_t color) {
    // Store circle for later clearing, but don't exceed our storage limit
    if (numPreviousCircles < MAX_STORED_CIRCLES) {
        previousCircles[numPreviousCircles] = {x, y, r, color, true};
        numPreviousCircles++;
    }
}

void drawPathPreview() {
    // 1. Check if path is valid
    if (path.numPoints <= 1 || path.numPoints > MAX_POINTS) {
        clearPreviousPathPreview(); // Clear any existing lines
        return; // Don't draw anything if no valid path
    }

    // 2. Clear previously drawn path lines
    clearPreviousPathPreview();

    // 3. Use the same coordinate system as the UI
    float padding = 6;
    float rectangleWidth = screen->width() / 2; // Same as in display.cpp
    float windowSize = rectangleWidth - 2*padding; // Same calculation as in drawUI
    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;
    
    // 4. Draw the path using the same coordinate transformation as the UI
    // Offset path points by the router's current position so the path moves with the router
    int16_t lastPx = -1, lastPy = -1;
    bool lastPosWasRapid = true;
    bool lastPointValid = false;

    for (int i = 0; i < path.numPoints; i++) {
        // Get path point coordinates
        float pathX = path.points[i].x;
        float pathY = path.points[i].y;
        float z = path.points[i].z;

        // Calculate relative position of path point to router's current position
        // This makes the path move with the router (like a heads-up display)
        float relativeX = pathX - pose.x;
        float relativeY = pathY - pose.y;

        // Transform using the same mapping as the UI
        // Map from relative coordinates to UI window coordinates
        float dx = mapF(relativeX, -xRange/2, xRange/2, -windowSize/2, windowSize/2);
        float dy = -mapF(relativeY, -yRange/2, yRange/2, -windowSize/2, windowSize/2);
        
        // Convert to screen coordinates
        int16_t px = centerX + dx;
        int16_t py = centerY + dy;
        
        // Check if point is within the UI rectangle bounds
        bool pointInBounds = (abs(dx) <= windowSize/2) && (abs(dy) <= windowSize/2);
        
        // Handle different features
        if (path.points[i].feature == DRILL && pointInBounds) {
            screen->drawCircle(px, py, 2, YELLOW); // Mark drill points
            lastPosWasRapid = true; // Treat as a "break" in the line
            lastPointValid = false; // Don't connect lines to drill points
        } else if (i > 0 && pointInBounds) {
            bool currentPosIsRapid = (z >= restHeight);
            
            if (lastPointValid) {
                if (!lastPosWasRapid && !currentPosIsRapid) {
                    // --- Cutting move (both points below rapid height) ---
                    screen->drawLine(lastPx, lastPy, px, py, GC9A01A_WEBWORK_GREEN);
                } else if ((lastPosWasRapid && !currentPosIsRapid) || (!lastPosWasRapid && currentPosIsRapid)) {
                    // --- Plunge or Retract move ---
                    screen->drawLine(lastPx, lastPy, px, py, 0x8410); // A dark gray
                }
                // Else: (lastPosWasRapid && currentPosIsRapid) -> Do not draw rapid-to-rapid lines
            }

            lastPosWasRapid = currentPosIsRapid;
            lastPx = px;
            lastPy = py;
            lastPointValid = true;
        } else if (pointInBounds) {
            // Set initial state for the very first point
            lastPosWasRapid = (z >= restHeight);
            lastPx = px;
            lastPy = py;
            lastPointValid = true;
        } else {
            // Point is out of bounds, break the line continuity
            lastPointValid = false;
        }
    }
}