#include "path_preview.h"
#include "globals.h"
#include "config.h"
#include "types.h"
#include "../math/geometry.h"

// Struct to store a dot's physical coordinates (not screen coordinates)
struct CachedDot {
    float physicalX, physicalY; // Physical world coordinates
    uint16_t color;
    bool valid;
};

// Struct to store a dot's final on-screen location for clearing
struct DrawnDot {
    int16_t x, y;
    bool valid;
};

// --- CONFIGURATION & LIMITS ---
static const int MAX_CACHED_DOTS = 1500;
static const int MAX_DRAWN_DOTS = 400;
const float DOTS_PER_MM = 0.5f; // One dot every 2mm

// Cache eviction parameters
static const float CACHE_REFRESH_DISTANCE_THRESHOLD = 100.0f; // mm - refresh when tool moves this far
static const float CACHE_REGION_RADIUS = 150.0f; // mm - cache dots within this radius of current position

// --- STATIC STORAGE ---
// Use DMAMEM attribute to place these large arrays in RAM2 instead of RAM1
DMAMEM static CachedDot cachedDots[MAX_CACHED_DOTS];
static int numCachedDots = 0;

DMAMEM static DrawnDot previousDots[MAX_DRAWN_DOTS];
static int numPreviousDots = 0;

// Cache validity tracking
static bool cacheValid = false;
static int lastPathNumPoints = -1;
static bool lastFullscreenMode = false;
static float lastCacheCenterX = 0.0f;
static float lastCacheCenterY = 0.0f;

// --- HELPER FUNCTIONS ---

/**
 * @brief Check if we need to refresh the cache based on tool movement
 */
bool shouldRefreshCache() {
    float dx = pose.x - lastCacheCenterX;
    float dy = pose.y - lastCacheCenterY;
    float distMoved = sqrt(dx * dx + dy * dy);
    return distMoved > CACHE_REFRESH_DISTANCE_THRESHOLD;
}

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
    numPreviousDots = 0;
}

void storeDrawnDot(int16_t x, int16_t y) {
    if (numPreviousDots < MAX_DRAWN_DOTS) {
        previousDots[numPreviousDots] = {x, y, true};
        numPreviousDots++;
    }
}

/**
 * @brief Smart precompute that caches dots in a region around current pose
 * Stores PHYSICAL coordinates, not screen coordinates
 */
void precomputePathDots() {
    numCachedDots = 0;
    
    // Store current cache center
    lastCacheCenterX = pose.x;
    lastCacheCenterY = pose.y;
    
    // Define cache region
    float cacheRadiusSq = CACHE_REGION_RADIUS * CACHE_REGION_RADIUS;
    
    const float DOT_INTERVAL_MM = 1.0f / DOTS_PER_MM;
    float accumulatedPhysicalDist = 0;
    float nextDotDistThreshold = 0;
    
    // Generate dots and cache those within the cache radius
    for (int i = 1; i < path.numPoints; i++) {
        if (numCachedDots >= MAX_CACHED_DOTS) break;
        
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
                // Calculate physical position of the dot
                float dot_x_phys = p0.x + t * dx_phys;
                float dot_y_phys = p0.y + t * dy_phys;
                
                // Calculate distance from current pose
                float dx_from_pose = dot_x_phys - lastCacheCenterX;
                float dy_from_pose = dot_y_phys - lastCacheCenterY;
                float distSq = dx_from_pose * dx_from_pose + dy_from_pose * dy_from_pose;
                
                // Cache if within cache radius - store PHYSICAL coordinates only
                if (distSq <= cacheRadiusSq) {
                    cachedDots[numCachedDots] = {
                        dot_x_phys,
                        dot_y_phys,
                        dotColor,
                        true
                    };
                    numCachedDots++;
                }
            }
            nextDotDistThreshold += DOT_INTERVAL_MM;
        }
        accumulatedPhysicalDist += physicalSegmentLength;
    }
    
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
    bool pathChanged = (path.numPoints != lastPathNumPoints);
    bool viewModeChanged = (pathPreviewFullScreen != lastFullscreenMode);
    bool movedTooFar = shouldRefreshCache();
    
    if (!cacheValid || pathChanged || viewModeChanged || movedTooFar) {
        precomputePathDots();
    }
    
    // --- 2. CLEAR PREVIOUS FRAME ---
    clearPreviousPathPreview();
    
    // --- 3. RENDER FROM CACHE ---
    // Calculate screen parameters (same as original code)
    int16_t centerX = screen->width() / 2;
    int16_t centerY = screen->height() / 2;
    
    float padding = 6;
    float rectangleWidth = screen->width() / 2;
    float rectWindowSize = rectangleWidth - 2 * padding;
    float rectMaxX = rectWindowSize / 2;
    float rectMaxY = rectWindowSize / 2;
    float rectMinX = -rectWindowSize / 2;
    float rectMinY = -rectWindowSize / 2;
    
    // Render each cached dot
    for (int i = 0; i < numCachedDots; i++) {
        if (numPreviousDots >= MAX_DRAWN_DOTS) break;
        if (!cachedDots[i].valid) continue;
        
        // Calculate position RELATIVE to current pose (exactly like original code)
        float relativeX = cachedDots[i].physicalX - pose.x;
        float relativeY = cachedDots[i].physicalY - pose.y;
        
        // Map to screen coordinates (exactly like original code)
        float dx = mapF(relativeX, -xRange/2, xRange/2, rectMinX, rectMaxX);
        float dy = -mapF(relativeY, -yRange/2, yRange/2, rectMinY, rectMaxY);
        
        int16_t px = centerX + dx;
        int16_t py = centerY + dy;
        
        // Check if in bounds (exactly like original code)
        bool isInBounds;
        if (pathPreviewFullScreen) {
            float distFromCenter = sqrt(pow(px - centerX, 2) + pow(py - centerY, 2));
            isInBounds = (distFromCenter <= screen->width() / 2 - 5);
        } else {
            isInBounds = (abs(dx) <= rectWindowSize / 2) && (abs(dy) <= rectWindowSize / 2);
        }
        
        if (isInBounds) {
            screen->drawPixel(px, py, cachedDots[i].color);
            storeDrawnDot(px, py);
        }
    }
}