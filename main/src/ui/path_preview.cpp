#include "path_preview.h"
#include "globals.h"
#include "config.h"
#include "types.h"
#include "../math/geometry.h"

// Structure to store a dot for clearing
struct PathDot {
    int16_t x, y;
    uint16_t color;
    bool valid;
};

// Structure to store pre-computed path segment data
struct PrecomputedSegment {
    float screenX, screenY;  // Screen coordinates (float for precision)
    float z;
    Feature feature;
    bool isRapid;
};

// Static storage for previously drawn dots
static const int MAX_STORED_DOTS = 300;
static PathDot previousDots[MAX_STORED_DOTS];
static int numPreviousDots = 0;

// Cached path data
static const int MAX_PRECOMPUTED_SEGMENTS = 500;
static PrecomputedSegment precomputedPath[MAX_PRECOMPUTED_SEGMENTS];
static int numPrecomputedSegments = 0;
static float cachedTotalPathLength = 0;
static int cachedTargetDots = 0;
static float cachedDotsPerPixel = 0;

// Cache invalidation tracking
static int lastPathNumPoints = -1;
static float lastPoseX = 0;
static float lastPoseY = 0;
static bool cacheValid = false;

void clearPreviousPathPreview() {
    for (int i = 0; i < numPreviousDots; i++) {
        if (previousDots[i].valid) {
            screen->drawPixel(previousDots[i].x, previousDots[i].y, BLACK);
        }
    }
    numPreviousDots = 0;
}

void storeDot(int16_t x, int16_t y, uint16_t color) {
    if (numPreviousDots < MAX_STORED_DOTS) {
        previousDots[numPreviousDots] = {x, y, color, true};
        numPreviousDots++;
    }
}

void invalidatePathCache() {
    cacheValid = false;
}

void precomputePathData() {
    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;

    float padding = 6;
    float rectangleWidth = screen->width() / 2; 
    float rectWindowSize = rectangleWidth - 2*padding; 
    float rectMaxX = rectWindowSize/2;
    float rectMaxY = rectWindowSize/2;
    float rectMinX = -rectWindowSize/2;
    float rectMinY = -rectWindowSize/2;

    // Adaptive skip based on path density
    int pointSkip = 1;
    if (path.numPoints > 800) pointSkip = 3;
    else if (path.numPoints > 400) pointSkip = 2;

    // First pass: Convert to screen coordinates and calculate total length
    numPrecomputedSegments = 0;
    cachedTotalPathLength = 0;
    float lastScreenX = 0, lastScreenY = 0;
    bool firstPoint = true;

    for (int i = 0; i < path.numPoints && numPrecomputedSegments < MAX_PRECOMPUTED_SEGMENTS; i += pointSkip) {
        float pathX = path.points[i].x;
        float pathY = path.points[i].y;
        float z = path.points[i].z;
        Feature feature = path.points[i].feature;

        float relativeX = pathX - pose.x;
        float relativeY = pathY - pose.y;

        float dx = mapF(relativeX, -xRange/2, xRange/2, rectMinX, rectMaxX);
        float dy = -mapF(relativeY, -yRange/2, yRange/2, rectMinY, rectMaxY);

        float screenX = centerX + dx;
        float screenY = centerY + dy;

        // Store precomputed segment
        precomputedPath[numPrecomputedSegments].screenX = screenX;
        precomputedPath[numPrecomputedSegments].screenY = screenY;
        precomputedPath[numPrecomputedSegments].z = z;
        precomputedPath[numPrecomputedSegments].feature = feature;
        precomputedPath[numPrecomputedSegments].isRapid = (z >= restHeight);

        // Calculate segment length for total path length
        if (!firstPoint && feature != DRILL) {
            float dist_dx = screenX - lastScreenX;
            float dist_dy = screenY - lastScreenY;
            cachedTotalPathLength += sqrt(dist_dx*dist_dx + dist_dy*dist_dy);
        }

        lastScreenX = screenX;
        lastScreenY = screenY;
        firstPoint = false;
        numPrecomputedSegments++;
    }

    // Calculate dots per pixel
    cachedTargetDots = 400;  // Adjustable
    if (path.numPoints > 1000) {
        cachedTargetDots = 350;
    } else if (path.numPoints < 200) {
        cachedTargetDots = 500;
    }
    
    cachedDotsPerPixel = (cachedTotalPathLength > 0) ? (float)cachedTargetDots / cachedTotalPathLength : 0;

    // Mark cache as valid
    lastPathNumPoints = path.numPoints;
    lastPoseX = pose.x;
    lastPoseY = pose.y;
    cacheValid = true;
}

void drawPathPreview() {
    if (path.numPoints <= 1 || path.numPoints > MAX_POINTS) {
        clearPreviousPathPreview(); 
        cacheValid = false;
        return; 
    }

    // Check if we need to recompute the path data
    bool poseChanged = (abs(pose.x - lastPoseX) > 0.01f || abs(pose.y - lastPoseY) > 0.01f);
    bool pathChanged = (path.numPoints != lastPathNumPoints);
    
    if (!cacheValid || pathChanged || poseChanged) {
        precomputePathData();
    }

    // Clear previous dots
    clearPreviousPathPreview();

    if (numPrecomputedSegments <= 1) return;

    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;

    float padding = 6;
    float rectangleWidth = screen->width() / 2; 
    float rectWindowSize = rectangleWidth - 2*padding;

    // Now render using precomputed data
    float accumulatedDistance = 0;
    float nextDotThreshold = 0;
    bool lastPointValid = false;
    float lastScreenX = 0, lastScreenY = 0;
    bool lastPosWasRapid = true;

    for (int i = 0; i < numPrecomputedSegments; i++) {
        float screenX = precomputedPath[i].screenX;
        float screenY = precomputedPath[i].screenY;
        Feature feature = precomputedPath[i].feature;
        bool currentPosIsRapid = precomputedPath[i].isRapid;

        int16_t px = (int16_t)screenX;
        int16_t py = (int16_t)screenY;

        bool pointInBounds;
        if (pathPreviewFullScreen) {
            float distFromCenter = sqrt(pow(px - centerX, 2) + pow(py - centerY, 2));
            pointInBounds = (distFromCenter <= screen->width()/2 - 5); 
        } else {
            int16_t dx = px - centerX;
            int16_t dy = py - centerY;
            pointInBounds = (abs(dx) <= rectWindowSize/2) && (abs(dy) <= rectWindowSize/2);
        }

        if (feature == DRILL) {
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
            float dist_dx = screenX - lastScreenX;
            float dist_dy = screenY - lastScreenY;
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
                
                int dotX = lastScreenX + (int)(t * dist_dx);
                int dotY = lastScreenY + (int)(t * dist_dy);
                
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
                
                nextDotThreshold += (1.0f / cachedDotsPerPixel);
            }

            lastPosWasRapid = currentPosIsRapid;
            lastScreenX = screenX;
            lastScreenY = screenY;
            lastPointValid = true;
            
        } else if (i == 0) {
            lastPosWasRapid = currentPosIsRapid;
            lastScreenX = screenX;
            lastScreenY = screenY;
            lastPointValid = true;
        }
    }
}