#ifndef TYPES_H
#define TYPES_H

// State values
typedef enum State {
	POWER_ON,
	MACHINE_X_ZERO,
	WORKSPACE_Z_ZERO,
	THICKNESS_SET,
	SELECTING_DESIGN,
	DESIGN_SELECTED,
	WORKSPACE_XY_ZERO,
	READY
} State;

// Coordinate
typedef struct Point {
	float x;
	float y;
	float z;
} Point;

#endif
