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
    if (path.numPoints <= 1 || path.numPoints > MAX_POINTS) {
        clearPreviousPathPreview(); 
        return; 
    }

    clearPreviousPathPreview();

    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;

    float padding = 6;
    float rectangleWidth = screen->width() / 2; 
    float rectWindowSize = rectangleWidth - 2*padding; 
    float rectMaxX = rectWindowSize/2;
    float rectMaxY = rectWindowSize/2;
    float rectMinX = -rectWindowSize/2;
    float rectMinY = -rectWindowSize/2;

    // Calculate total path length first
    float totalPathLength = 0;
    int16_t lastPx = -1, lastPy = -1;
    bool firstPointProcessed = false;
    
    for (int i = 0; i < path.numPoints; i++) {
        float pathX = path.points[i].x;
        float pathY = path.points[i].y;
        float relativeX = pathX - pose.x;
        float relativeY = pathY - pose.y;
        
        float dx = mapF(relativeX, -xRange/2, xRange/2, rectMinX, rectMaxX);
        float dy = -mapF(relativeY, -yRange/2, yRange/2, rectMinY, rectMaxY);
        
        int16_t px = centerX + dx;
        int16_t py = centerY + dy;
        
        if (firstPointProcessed) {
            int dist_dx = px - lastPx;
            int dist_dy = py - lastPy;
            totalPathLength += sqrt(dist_dx*dist_dx + dist_dy*dist_dy);
        }
        
        lastPx = px;
        lastPy = py;
        firstPointProcessed = true;
    }
    
    // Target number of dots for the entire path
    const int TARGET_TOTAL_DOTS = 400;
    float dotsPerPixel = (totalPathLength > 0) ? (float)TARGET_TOTAL_DOTS / totalPathLength : 0;
    
    // Now draw with standardized spacing
    lastPx = -1;
    lastPy = -1;
    bool lastPosWasRapid = true;
    bool lastPointValid = false;
    float accumulatedDistance = 0;
    float nextDotThreshold = 0;

    for (int i = 0; i < path.numPoints; i++) {
        float pathX = path.points[i].x;
        float pathY = path.points[i].y;
        float z = path.points[i].z;

        float relativeX = pathX - pose.x;
        float relativeY = pathY - pose.y;

        float dx = mapF(relativeX, -xRange/2, xRange/2, rectMinX, rectMaxX);
        float dy = -mapF(relativeY, -yRange/2, yRange/2, rectMinY, rectMaxY);

        int16_t px = centerX + dx;
        int16_t py = centerY + dy;

        bool pointInBounds;
        if (pathPreviewFullScreen) {
            float distFromCenter = sqrt(pow(px - centerX, 2) + pow(py - centerY, 2));
            pointInBounds = (distFromCenter <= screen->width()/2 - 5); 
        } else {
            pointInBounds = (abs(dx) <= rectWindowSize/2) && (abs(dy) <= rectWindowSize/2);
        }

        if (path.points[i].feature == DRILL) {
            if (pointInBounds) {
                screen->drawPixel(px, py, YELLOW);         
                screen->drawPixel(px-1, py, YELLOW);       
                screen->drawPixel(px+1, py, YELLOW);       
                screen->drawPixel(px, py-1, YELLOW);       
                screen->drawPixel(px, py+1, YELLOW);       

                storeDot(px, py, YELLOW);
                storeDot(px-1, py, YELLOW);
                storeDot(px+1, py, YELLOW);
                storeDot(px, py-1, YELLOW);
                storeDot(px, py+1, YELLOW);
            }

            lastPosWasRapid = true; 
            lastPointValid = false;
            accumulatedDistance = 0;
            nextDotThreshold = 0;
            
        } else if (i > 0 && lastPointValid) {
            bool currentPosIsRapid = (z >= restHeight);
            
            int dist_dx = px - lastPx;
            int dist_dy = py - lastPy;
            float segmentLength = sqrt(dist_dx*dist_dx + dist_dy*dist_dy);
            
            accumulatedDistance += segmentLength;

            // Determine color based on move type
            uint16_t dotColor;
            if (!lastPosWasRapid && !currentPosIsRapid) {
                dotColor = GC9A01A_WEBWORK_GREEN;
            } else if (lastPosWasRapid && currentPosIsRapid) {
                dotColor = CYAN;
            } else {
                dotColor = 0x8410;
            }

            // Draw dots at standardized intervals
            while (accumulatedDistance >= nextDotThreshold && numPreviousDots < MAX_STORED_DOTS - 1) {
                float t = (nextDotThreshold - (accumulatedDistance - segmentLength)) / segmentLength;
                t = constrain(t, 0.0f, 1.0f);
                
                int dotX = lastPx + (int)(t * dist_dx);
                int dotY = lastPy + (int)(t * dist_dy);
                
                bool dotInBounds;
                if (pathPreviewFullScreen) {
                    float distFromCenter = sqrt(pow(dotX - centerX, 2) + pow(dotY - centerY, 2));
                    dotInBounds = (distFromCenter <= screen->width()/2 - 5);
                } else {
                    int16_t dotDx = dotX - centerX;
                    int16_t dotDy = dotY - centerY;
                    dotInBounds = (abs(dotDx) <= rectWindowSize/2) && (abs(dotDy) <= rectWindowSize/2);
                }
                
                if (dotInBounds) {
                    screen->drawPixel(dotX, dotY, dotColor);
                    storeDot(dotX, dotY, dotColor);
                }
                
                nextDotThreshold += (1.0f / dotsPerPixel);
            }

            lastPosWasRapid = currentPosIsRapid;
            lastPx = px;
            lastPy = py;
            lastPointValid = true;
            
        } else if (i == 0) {
            lastPosWasRapid = (z >= restHeight);
            lastPx = px;
            lastPy = py;
            lastPointValid = true;
        }
    }
}