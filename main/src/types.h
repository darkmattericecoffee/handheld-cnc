#ifndef TYPES_H
#define TYPES_H

#include "config.h"

// State values
typedef enum State {
	POWER_ON,
	MACHINE_X_ZERO,
	WORKSPACE_Z_ZERO,
	RESET,
	THICKNESS_SET,
	DOC_SELECTED,
	CALIBRATION,
	CALIBRATION_ADVANCE,
	TYPE_SELECTED,
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

// Calibration parameters
typedef struct CalParams {
	float x = 25.4 / CPI;		// x scalar (default based on sensor CPI)
	float y = 25.4 / CPI;		// y scalar (default based on sensor CPI)
	float r = 0.0f;				// sensor rotation
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
