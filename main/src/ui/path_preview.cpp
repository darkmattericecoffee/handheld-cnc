#include "path_preview.h"
#include "globals.h" // Access to 'screen' and 'path'
#include "config.h"  // Access to 'MAX_POINTS', 'GC9A01A_WEBWORK_GREEN', etc.
#include "types.h"   // Access to 'restHeight' and 'Feature' enum
#include "../math/geometry.h" // Access to 'mapF' function

// Structure to store a dot for clearing
struct PathDot {
    int16_t x, y;
    uint16_t color;
    bool valid;
};

// Static storage for previously drawn dots (much more efficient than lines/rects)
static const int MAX_STORED_DOTS = 300;
static PathDot previousDots[MAX_STORED_DOTS];
static int numPreviousDots = 0;

void clearPreviousPathPreview() {
    // Ultra-fast clear: just clear individual dots (single pixel operations)
    // This is much faster and less intrusive than fillRect
    for (int i = 0; i < numPreviousDots; i++) {
        if (previousDots[i].valid) {
            screen->drawPixel(previousDots[i].x, previousDots[i].y, BLACK);
        }
    }
    numPreviousDots = 0;  // Reset the count
}

void storeDot(int16_t x, int16_t y, uint16_t color) {
    // Store dot for later clearing, but don't exceed our storage limit
    if (numPreviousDots < MAX_STORED_DOTS) {
        previousDots[numPreviousDots] = {x, y, color, true};
        numPreviousDots++;
    }
}

void drawPathPreview() {
    // 1. Check if path is valid
    if (path.numPoints <= 1 || path.numPoints > MAX_POINTS) {
        clearPreviousPathPreview(); // Clear any existing lines
        return; // Don't draw anything if no valid path
    }

    // 2. Clear previously drawn path lines (fast area clear)
    clearPreviousPathPreview();

    // 3. Configure coordinate system
    //    We ALWAYS use the rectangle's coordinate system for scaling
    //    to ensure it matches the target circle in drawUI.
    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;
    
    float padding = 6;
    float rectangleWidth = screen->width() / 2; // Same as in display.cpp
    float rectWindowSize = rectangleWidth - 2*padding; // Same calculation as in drawUI
    float rectMaxX = rectWindowSize/2;
    float rectMaxY = rectWindowSize/2;
    float rectMinX = -rectWindowSize/2;
    float rectMinY = -rectWindowSize/2;
    
    // 4. Draw the path using coordinate transformation
    // Offset path points by the router's current position so the path moves with the router
    // OPTIMIZATION: Only draw every Nth point to reduce drawing operations
    int pointSkip = max(1, path.numPoints / 150); // Limit to ~150 lines max for performance
    
    int16_t lastPx = -1, lastPy = -1;
    bool lastPosWasRapid = true;
    bool lastPointValid = false;

    for (int i = 0; i < path.numPoints; i += pointSkip) {  // Skip points for performance
        // Get path point coordinates
        float pathX = path.points[i].x;
        float pathY = path.points[i].y;
        float z = path.points[i].z;

        // Calculate relative position of path point to router's current position
        // This makes the path move with the router (like a heads-up display)
        float relativeX = pathX - pose.x;
        float relativeY = pathY - pose.y;

        // [MODIFIED] Transform coordinates using the RECTANGLE mode scaling ALWAYS.
        // This ensures 1:1 scaling correspondence with the target circle in drawUI.
        float dx, dy;
        dx = mapF(relativeX, -xRange/2, xRange/2, rectMinX, rectMaxX);
        dy = -mapF(relativeY, -yRange/2, yRange/2, rectMinY, rectMaxY);
        
        // Convert to absolute screen coordinates
        int16_t px = centerX + dx;
        int16_t py = centerY + dy;
        
        // [MODIFIED] Check if point is within bounds
        // This is the ONLY place where pathPreviewFullScreen matters.
        bool pointInBounds;
        if (pathPreviewFullScreen) {
            // Full screen: check if within circular display bounds (rough approximation)
            // We use px and py (absolute screen coords) for the check.
            float distFromCenter = sqrt(pow(px - centerX, 2) + pow(py - centerY, 2));
            pointInBounds = (distFromCenter <= screen->width()/2 - 5); // 5 pixel margin
        } else {
            // Rectangle mode: check if within rectangle bounds
            // We can check dx/dy (relative coords) against the rectWindowSize.
            pointInBounds = (abs(dx) <= rectWindowSize/2) && (abs(dy) <= rectWindowSize/2);
        }
        
        // Handle different features
        if (path.points[i].feature == DRILL && pointInBounds) {
            // Draw drill points as a small cross pattern (faster than circle)
            screen->drawPixel(px, py, YELLOW);         // center
            screen->drawPixel(px-1, py, YELLOW);       // left
            screen->drawPixel(px+1, py, YELLOW);       // right
            screen->drawPixel(px, py-1, YELLOW);       // up
            screen->drawPixel(px, py+1, YELLOW);       // down
            
            // Store all drill pixels for clearing
            storeDot(px, py, YELLOW);
            storeDot(px-1, py, YELLOW);
            storeDot(px+1, py, YELLOW);
            storeDot(px, py-1, YELLOW);
            storeDot(px, py+1, YELLOW);
            
            lastPosWasRapid = true; // Treat as a "break" in the line
            lastPointValid = false; // Don't connect lines to drill points
        } else if (i > 0 && pointInBounds) {
            bool currentPosIsRapid = (z >= restHeight);
            
            if (lastPointValid) {
                uint16_t dotColor = BLACK; // Default: don't draw
                
                if (!lastPosWasRapid && !currentPosIsRapid) {
                    // --- Cutting move (both points below rapid height) ---
                    dotColor = GC9A01A_WEBWORK_GREEN;
                } else if ((lastPosWasRapid && !currentPosIsRapid) || (!lastPosWasRapid && currentPosIsRapid)) {
                    // --- Plunge or Retract move ---
                    dotColor = 0x8410; // A dark gray
                }
                
                if (dotColor != BLACK) {
                    // Draw stippled line as dots every few pixels for ultra-fast drawing
                    int dist_dx = px - lastPx; // Use 'dist_dx' to avoid shadowing
                    int dist_dy = py - lastPy; // Use 'dist_dy' to avoid shadowing
                    float distance = sqrt(dist_dx*dist_dx + dist_dy*dist_dy);
                    int numDots = max(1, (int)(distance / 3)); // Dot every ~3 pixels
                    
                    for (int d = 0; d < numDots && numPreviousDots < MAX_STORED_DOTS - 1; d++) {
                        float t = (float)d / (float)numDots;
                        int dotX = lastPx + (int)(t * dist_dx);
                        int dotY = lastPy + (int)(t * dist_dy);
                        screen->drawPixel(dotX, dotY, dotColor);
                        storeDot(dotX, dotY, dotColor);
                    }
                }
                // Else: (lastPosWasRapid && currentPosIsRapid) -> Do not draw rapid-to-rapid lines
            }

            lastPosWasRapid = currentPosIsRapid;
            lastPx = px;
            lastPy = py;
            lastPointValid = true;
        } else if (i == 0 && pointInBounds) { // Handle first point
            // Set initial state for the very first point
            lastPosWasRapid = (z >= restHeight);
            lastPx = px;
            lastPy = py;
            lastPointValid = true;
        } else if (!pointInBounds) {
            // Point is out of bounds, break the line continuity
            lastPointValid = false;
        }
    }
}