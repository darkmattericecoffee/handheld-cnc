#include "geometry.h"
#include "../globals.h"
#include "../config.h"
#include <Arduino.h>

float myDist(float x1, float y1, float x2, float y2) {
	// Calculate the Euclidean distance between two points
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

float clamp(float val, float min, float max) {
	// Clamp a value between min and max
	if (val < min) {
		return min;
	}
	if (val > max) {
		return max;
	}
	return val;
}

float principalAngleRad(float x) {
	// Returns x mapped between 0 and PI
	// This assumes that the line is infinite, so angle may be 180 degrees
	// different from the original value x.
	// Ex: 3 PI/2 -> PI/2
	while (x > PI) {
		x -= PI;
	}
	while (x < 0) {
		x += PI;
	}
	return x;
}

float mapF(long x, float in_min, float in_max, float out_min, float out_max) {
	// Maps a float value from one range to another
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float signedDist(float xr, float yr, float xg, float yg, float th) {
	// Calculate the signed distance between goal point and line of gantry
	// Note: if the distance is:
	//    < 0 - the point is in front of the gantry (it is yet to be passed)
	//    > 0 - the point is behind the gantry (it has been passed)
	float m = tan(th);
	float b = yr - m * xr;
	float A = m;
	float B = -1;
	float C = b;

	return (A * xg + B * yg + C) / sqrt(pow(A, 2) + pow(B, 2));
}

float angleFrom(Point a, Point b) {
	// Returns the angle (rads) between the gantry and the 
	// line connecting points a and b.
	float th1 = principalAngleRad(atan2f(b.y - a.y, b.x - a.x));

	// Using yaw here is a bit odd, since yaw is 0 when the router is in the
	// original orientation, which would actually be 90 degrees from +x. However,
	// we actually care about the angle of the gantry, which will be 0 degrees from +x
	// when the yaw is 0, so this works ok.
	extern float estYaw;  // Declared in globals.h
	float th2 = principalAngleRad(estYaw);

	return abs(th1 - th2);
}

float desPosIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4) {
	// Returns the desired stepperX position such that the cutting tool intersects 
	// the infinite line from (x3,y3) to (x4,y4).
	// If the gantry does not intersect the line, this returns NAN.
	// (xc,yc) is the current position of the router, and th is the yaw.
	
	// Calculate gantry endpoints
	float x1 = xc - (cosf(th) * gantryLength / 2);
	float y1 = yc - (sinf(th) * gantryLength / 2);
	float x2 = xc + (cosf(th) * gantryLength / 2);
	float y2 = yc + (sinf(th) * gantryLength / 2);
	
	// Calculate intersection using line-line intersection formula
	float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

	// Check for parallel lines (denominator is zero)
	if (den == 0) {
		// Serial.println("parallel");
		return NAN;
	}

	float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
	
	// Check if the intersection point is on the gantry
	if (t < 0 || t > 1) {
		// Serial.println("not on gantry");
		return NAN;
	}
	
	// Calculate intersection point
	float x = x1 + t * (x2 - x1);
	float y = y1 + t * (y2 - y1);

	// Convert to gantry coordinates
	float dx = x - xc;
	float dy = y - yc;

	return dx * cosf(th) + dy * sinf(th);
}

float desPosClosestToIntersect(float xc, float yc, float th, float x3, float y3, float x4, float y4) {
	// Returns the desired stepperX position such that the cutting tool intersects 
	// the infinite line from (x3,y3) to (x4,y4).
	// If the gantry does not intersect the line, this returns the position
	// that gets the tool closest to intersecting the line.
	
	// Calculate gantry endpoints
	float x1 = xc - (cosf(th) * gantryLength / 2);
	float y1 = yc - (sinf(th) * gantryLength / 2);
	float x2 = xc + (cosf(th) * gantryLength / 2);
	float y2 = yc + (sinf(th) * gantryLength / 2);
	
	float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

	// Check for parallel lines (denominator is zero)
	if (den == 0) {
		return stepperX.currentPosition() * 1.0f / Conv;
	}

	float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
	
	// Calculate intersection point
	float x = x1 + t * (x2 - x1);
	float y = y1 + t * (y2 - y1);

	// Convert to gantry coordinates
	float dx = x - xc;
	float dy = y - yc;

	float desiredPos = dx * cosf(th) + dy * sinf(th);
	float maxPos = (gantryLength / 2.0) - xBuffer;

	return clamp(desiredPos, -maxPos, maxPos);
}

float desiredPosition(float dX, float dY, float theta) {
	// Calculate the desired position for the tool using Sanzhar equation
	return (dX + tanf(theta) * dY) * cosf(theta);
	// This can also be written as:
	// return dX * cosf(theta) + dY * sinf(theta);
}
