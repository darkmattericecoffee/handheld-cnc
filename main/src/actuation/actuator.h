#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "../types.h"
#include "../config.h"
#include "../globals.h"
#include "pid.h"
#include "motors.h"

class ActuationController {
public:
	ActuationController();

	// Compute actuator adjustments (in local device frame)
	void update(
		long deltaTime,
		Point goal,
		RouterPose currentPose
	);

private:
	PIDController pidX, pidY, pidZ;
	const float actuatorRangeX = xRange;
	const float actuatorRangeY = yRange;
	const float actuatorRangeZ = zRange;
};

#endif