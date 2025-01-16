#ifndef TYPES_H
#define TYPES_H

#include "config.h"

// State values
typedef enum State {
	POWER_ON,
	MACHINE_X_ZERO,
	WORKSPACE_Z_ZERO,
	THICKNESS_SET,
	TYPE_SELECTED,
	SELECTING_DESIGN,
	DESIGN_SELECTED,
	WORKSPACE_XY_ZERO,
	READY
} State;

// Type of design
typedef enum DesignType {
	PRESET,
	FROM_FILE
 } DesignType;

// Feature type
typedef enum Feature {
	// TODO: add types for travel, contour, pocket, etc.
	NORMAL,
	HOLE,
} Feature;

// Router pose
typedef struct RouterPose {
	float x;			// x position
	float y;			// y position
	float yaw;			// orientation
	// Future additions like velocity, acceleration, etc.
} RouterPose;

// Coordinate
typedef struct Point {
	float x;
	float y;
	float z;
} Point;

// Path
typedef struct Path {
	// TODO: add other properties such as angle, numPoints, etc. (more info in notebook)
	Point points[MAX_POINTS];
	int direction = 1;
	Feature feature = NORMAL;
	int numPoints;
} Path;

#endif
