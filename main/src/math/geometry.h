#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <AccelStepper.h>
#include "../types.h"

// Basic geometric operations
float myDist(float x1, float y1, float x2, float y2);
float clamp(float val, float min, float max);
float principalAngleRad(float x);
float mapF(long x, float in_min, float in_max, float out_min, float out_max);

// Path and angle calculations
float signedDist(float xr, float yr, float xg, float yg, float th);
float angleFrom(Point a, Point b);

// Gantry intersection calculations
float desPosIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4);
float desPosClosestToIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4);
float desiredPosition(float dX, float dY, float theta);

#endif
