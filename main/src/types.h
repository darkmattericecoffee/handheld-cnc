#ifndef TYPES_H
#define TYPES_H

#include "config.h"

// State values
typedef enum State {
	POWER_ON,
	MACHINE_XY_ZERO,
	WORKSPACE_Z_ZERO,
	THICKNESS_SET,
	DOC_SELECTED,
	CALIBRATION,
	CALIBRATION_ADVANCE,
	TYPE_SELECTED,
	SELECTING_DESIGN,
	DESIGN_SELECTED,
	WORKSPACE_XY_ZERO,
	READY
} State;

typedef enum CutState {
	NOT_CUT_READY,
	CUT_READY,
	CUTTING,
	PLUNGING,
	RETRACTING
} CutState;

// Type of design
typedef enum DesignType {
	PRESET,
	FROM_FILE
 } DesignType;

// Feature type
typedef enum Feature {
	// TODO: add types for travel, contour, pocket, etc.
	NORMAL,
	DRILL,
} Feature;

// Router pose
typedef struct RouterPose {
	float x;			// x position
	float y;			// y position
	float yaw;			// orientation
	// Future additions like velocity, acceleration, etc.
} RouterPose;

class Position {
	public:
		Position(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {
			clamp();
		}
	
		void set(float newX, float newY, float newZ) {
			x = newX;
			y = newY;
			z = newZ;
			clamp();
		}
	
		void setX(float newX) {
			x = newX;
			clamp();
		}
	
		void setY(float newY) {
			y = newY;
			clamp();
		}
	
		void setZ(float newZ) {
			z = newZ;
			clamp();
		}
	
		float getX() const { return x; }
		float getY() const { return y; }
		float getZ() const { return z; }
	
	private:
		float x, y, z;
	
		void clamp() {
			x = constrainValue(x, -xRange / 2, xRange / 2);
			y = constrainValue(y, -yRange / 2, yRange / 2);
			z = constrainValue(z, 0, zRange);
		}
	
		float constrainValue(float value, float min, float max) {
			if (value < min) return min;
			if (value > max) return max;
			return value;
		}
};

// Calibration parameters
typedef struct CalParams {
	float x = 0.01f;		// x scalar
	float y = 0.01f;		// y scalar
	float r = 0.0f;			// sensor rotation
} CalParams;

// Coordinate
typedef struct Point {
	float x;
	float y;
	float z;
} Point;

// Path
typedef struct Path {
	// TODO: add other properties such as angle, etc. (more info in notebook)
	Point points[MAX_POINTS] = {NAN};
	Feature feature = NORMAL;
	int numPoints = 0;
	float minZ = 0.0f;
} Path;

#endif
