#include "actuator.h"

ActuationController::ActuationController() 
    : pidX(1.0, 0.1, 0.01), pidY(1.0, 0.1, 0.01), pidZ(1.0, 0.1, 0.01) {
}

// Compute actuator adjustments (call in control loop)
void ActuationController::update(
	long deltaTime,
	Point goal,
	RouterPose currentPose
) {
	// Global position error
	float errX_global = goal.x - currentPose.x;
	float errY_global = goal.y - currentPose.y;
	float errZ_global = goal.z;								// TODO: does this need to be implemented differently?

	// Coordinate transform (global error -> local actuator frame)
	float errX_local = errX_global * cosf(currentPose.yaw) + errY_global * sinf(currentPose.yaw);
	float errY_local = -errX_global * sinf(currentPose.yaw) + errY_global * cosf(currentPose.yaw);

	// PID control
	// desPos.setX(pidX.compute(errX_local, deltaTime));
	// desPos.setY(pidY.compute(errY_local, deltaTime));
	// desPos.setZ(pidZ.compute(errZ_global, deltaTime));		// TODO: does this need to be implemented differently?

	// Normal control (without PID)
	desPos.setX(errX_local);
	desPos.setY(errY_local);
	desPos.setZ(errZ_global);								// TODO: does this need to be implemented differently?
}
