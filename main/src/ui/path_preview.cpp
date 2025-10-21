#include "path_preview.h"
#include "globals.h"
#include "config.h"
#include "types.h"
#include "../math/geometry.h"

// Struct to store a dot's pre-computed screen coordinates
struct CachedDot {
    int16_t x, y; // Base screen coordinates (relative to a zeroed pose)
    uint16_t color;
    bool valid;
};

// Struct to store a dot's final on-screen location for clearing
struct DrawnDot {
    int16_t x, y;
    bool valid;
};


// --- CONFIGURATION & LIMITS ---

// CACHE: Stores pre-computed dots for the ENTIRE path. Increase if very long paths are clipped.
static const int MAX_CACHED_DOTS = 2000;

// RENDER LIMIT: The maximum number of dots drawn ON SCREEN per frame to ensure performance.
static const int MAX_DRAWN_DOTS = 400;

// VISUAL DENSITY: Controls how many dots are generated per millimeter of real-world travel.
const float DOTS_PER_MM = 0.5f; // One dot every 2mm


// --- STATIC STORAGE ---

// The main cache for the entire path
static CachedDot cachedDots[MAX_CACHED_DOTS];
static int numCachedDots = 0;

// Stores the locations of dots drawn in the last frame for clearing
static DrawnDot previousDots[MAX_DRAWN_DOTS];
static int numPreviousDots = 0;

// Cache validity tracking
static bool cacheValid = false;
static int lastPathNumPoints = -1;
static bool lastFullscreenMode = false;


// --- CACHE & RENDER LOGIC ---

void invalidatePathCache() {
    cacheValid = false;
}

void clearPreviousPathPreview() {
    for (int i = 0; i < numPreviousDots; i++) {
        if (previousDots[i].valid) {
            screen->drawPixel(previousDots[i].x, previousDots[i].y, BLACK);
        }
    }
    numPreviousDots = 0; // Reset for the next frame
}

void storeDrawnDot(int16_t x, int16_t y) {
    if (numPreviousDots < MAX_DRAWN_DOTS) {
        previousDots[numPreviousDots] = {x, y, true};
        numPreviousDots++;
    }
}

/**
 * @brief Performs the one-time, expensive calculation of all dot positions for the path.
 * This populates the `cachedDots` array.
 */
void precomputePathDots() {
    numCachedDots = 0;
    const float DOT_INTERVAL_MM = 1.0f / DOTS_PER_MM;

    float accumulatedPhysicalDist = 0;
    float nextDotDistThreshold = 0;

    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;
    float padding = 6;
    float rectangleWidth = screen->width() / 2;
    float rectWindowSize = rectangleWidth - 2 * padding;

    for (int i = 1; i < path.numPoints; i++) {
        if (numCachedDots >= MAX_CACHED_DOTS) break; // Stop if cache is full

        Point p0 = path.points[i - 1];
        Point p1 = path.points[i];

        if (p0.feature == DRILL) continue;

        float dx_phys = p1.x - p0.x;
        float dy_phys = p1.y - p0.y;
        float physicalSegmentLength = sqrt(dx_phys * dx_phys + dy_phys * dy_phys);

        if (physicalSegmentLength < 0.001f) continue;

        bool isRapid = (p0.z >= restHeight && p1.z >= restHeight);
        uint16_t dotColor = isRapid ? CYAN : GC9A01A_WEBWORK_GREEN;

        while (nextDotDistThreshold <= accumulatedPhysicalDist + physicalSegmentLength) {
             if (numCachedDots >= MAX_CACHED_DOTS) break;

            float distanceToNextDot = nextDotDistThreshold - accumulatedPhysicalDist;
            float t = distanceToNextDot / physicalSegmentLength;

            if (t >= 0.0f && t <= 1.0f) {
                // Interpolate physical position of the dot
                float dot_x_phys = p0.x + t * dx_phys;
                float dot_y_phys = p0.y + t * dy_phys;

                // Map to screen coordinates (assuming pose is at 0,0 for the cache)
                float dx_screen = mapF(dot_x_phys, -xRange / 2, xRange / 2, -rectWindowSize / 2, rectWindowSize / 2);
                float dy_screen = -mapF(dot_y_phys, -yRange / 2, yRange / 2, -rectWindowSize / 2, rectWindowSize / 2);

                // Store in cache
                cachedDots[numCachedDots] = {
                    (int16_t)(centerX + dx_screen),
                    (int16_t)(centerY + dy_screen),
                    dotColor,
                    true
                };
                numCachedDots++;
            }
            nextDotDistThreshold += DOT_INTERVAL_MM;
        }
        accumulatedPhysicalDist += physicalSegmentLength;
    }

    // Mark the cache as valid for the current path and settings
    cacheValid = true;
    lastPathNumPoints = path.numPoints;
    lastFullscreenMode = pathPreviewFullScreen;
}

void drawPathPreview() {
    if (path.numPoints <= 1) {
        clearPreviousPathPreview();
        return;
    }

    // --- 1. VALIDATE CACHE ---
    // Re-compute only if the path or a major view setting has changed.
    bool pathChanged = (path.numPoints != lastPathNumPoints);
    bool viewModeChanged = (pathPreviewFullScreen != lastFullscreenMode);
    if (!cacheValid || pathChanged || viewModeChanged) {
        precomputePathDots();
    }

    // --- 2. CLEAR PREVIOUS FRAME ---
    clearPreviousPathPreview();

    // --- 3. RENDER FROM CACHE (FAST) ---
    // Calculate the current offset based on the tool's pose
    float pose_dx_screen = mapF(pose.x, -xRange / 2, xRange / 2, -screen->width()/4, screen->width()/4);
    float pose_dy_screen = -mapF(pose.y, -yRange / 2, yRange / 2, -screen->height()/4, screen->height()/4);

    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;
    float padding = 6;
    float rectangleWidth = screen->width() / 2;
    float rectWindowSize = rectangleWidth - 2 * padding;

    for (int i = 0; i < numCachedDots; i++) {
        // Stop drawing if we hit the on-screen dot limit for this frame
        if (numPreviousDots >= MAX_DRAWN_DOTS) break;
        
        if (!cachedDots[i].valid) continue;

        // Apply the current tool pose offset to the cached screen coordinate
        int16_t finalX = cachedDots[i].x - (int16_t)pose_dx_screen;
        int16_t finalY = cachedDots[i].y - (int16_t)pose_dy_screen;

        // Check if the final dot position is within the visible bounds
        bool isInBounds;
        if (pathPreviewFullScreen) {
            float distFromCenter = sqrt(pow(finalX - centerX, 2) + pow(finalY - centerY, 2));
            isInBounds = (distFromCenter <= screen->width() / 2 - 5);
        } else {
            float dx_from_center = finalX - centerX;
            float dy_from_center = finalY - centerY;
            isInBounds = (abs(dx_from_center) <= rectWindowSize / 2) && (abs(dy_from_center) <= rectWindowSize / 2);
        }

        if (isInBounds) {
            screen->drawPixel(finalX, finalY, cachedDots[i].color);
            storeDrawnDot(finalX, finalY);
        }
    }
}