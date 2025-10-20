#include "actuator.h"

ActuationController::ActuationController(Position &dp)
    : desPos(&dp), pidX(1.0, 0.1, 0.01), pidY(1.0, 0.1, 0.01), pidZ(1.0, 0.1, 0.01) {
}

// Compute actuator adjustments (call in control loop)
/**
 * @brief Updates the actuation controller to compute desired positions based on the goal and current pose.
 * 
 * This function calculates the global position errors in X, Y, and Z directions relative to the goal and current router pose.
 * It then transforms these errors into the local actuator frame using the current yaw angle.
 * The desired positions are set directly without PID control, using the transformed errors.
 * Note: For this build, the validMotion object is inverted (swapping and negating errX_local and errY_local) to flip the axes,
 * as there is a bug where X and Y are actually inversed.
 * 
 * @param deltaTime The time elapsed since the last update, in milliseconds.
 * @param goal The target point in global coordinates.
 * @param currentPose The current pose of the router, including position and yaw.
 */
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
	// Uncomment for your code: validMotion = desPos->set(errX_local, errY_local, errZ_global);
	// TEMPORARY FIX FOR AXIS INVERSION BUG (proably in my compass build setup):
	validMotion = desPos->set(-errY_local, -errX_local, errZ_global);
}
