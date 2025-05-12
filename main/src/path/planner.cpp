#include "planner.h"

// Load gCode path (pre-processed into a vector of GCodePoints)
void TrajectoryGenerator::setPath() {
	segmentTime = 0.0;
	currentTime = 0.0;
	current_point_idx = 0;
}
	
// Get desired position at current time (call in control loop)
void TrajectoryGenerator::update(long deltaTime, Point& goal) {
	// TODO: bring in buffers of the path instead of just one point
	// TODO: check if path is finished
	// if (path.empty()) return;

	currentTime += float(deltaTime) / float(1'000'000);		// convert to seconds

	// Find the segment we're in
	while (current_point_idx < path.numPoints - 1) {
		// TODO: careful of infinite loops
		float feedrate = path.points[current_point_idx].f;

		float segmentDistance = sqrt(
			pow(path.points[current_point_idx + 1].x - path.points[current_point_idx].x, 2) +
			pow(path.points[current_point_idx + 1].y - path.points[current_point_idx].y, 2) +
			pow(path.points[current_point_idx + 1].z - path.points[current_point_idx].z, 2)
		);
		// This is the time at which the segment will be completed
		segmentTime = segmentDistance / feedrate;
		// Serial.printf("tCurr:%f, tSeg:%f, f:%f\n", currentTime, segmentTime, feedrate);

		if (currentTime <= segmentTime) {
			// Linear interpolation within the segment
			float t = currentTime / segmentTime;
			goal.x = path.points[current_point_idx].x + t * (path.points[current_point_idx + 1].x - path.points[current_point_idx].x);
			goal.y = path.points[current_point_idx].y + t * (path.points[current_point_idx + 1].y - path.points[current_point_idx].y);
			goal.z = path.points[current_point_idx].z + t * (path.points[current_point_idx + 1].z - path.points[current_point_idx].z);
			// TODO: add feedrate and feature type to goal
			return;
		} else {
			currentTime = 0.0;			// reset time for next segment
			current_point_idx++;
			// Serial.printf("Current point index: %i\n", current_point_idx);
		}
	}

	// TODO: Once end of path has been reached, handle this properly
	// X_des = path.back().X;
	// Y_des = path.back().Y;
	// Z_des = path.back().Z;
}