#include "path-execution.h"
#include "../config.h"
#include "../globals.h"
#include "../motors/motors.h"
#include "../math/geometry.h"
#include "../io/logging.h"
#include "../ui/display.h"
#include "../ui/encoder.h"

bool prevChecks[4] = {false};

bool performSafetyChecks() {
	if (digitalRead(LIMIT_MACH_X0) == LOW) {
		stopStepperX();
		stepperZ.moveTo(Conv*restHeight);
		while (stepperZ.distanceToGo() != 0) {
			stepperZ.run();
		}
		Serial.println("X limit reached");
		encoderDesignType();
		return false;
	}

	if (digitalRead(LIMIT_MACH_Z0) == LOW) {
		stopStepperZ();
		stopStepperX();
		Serial.println("Z limit reached");
		encoderDesignType();
		return false;
	}

	return true;
}

bool advance(Point goal, Point next, bool autoAdvance=false) {
	// Move through point indeces as needed
	if (paths[current_path_idx].direction * signedDist(pose,next) > 0 || autoAdvance) {
		// If next point is behind router, it becomes the new goal.
		current_point_idx++;
		// if (autoAdvance) Serial.println("Auto-advanced!");

		bool lastPoint = false;
		if (paths[current_path_idx].feature == NORMAL) {
			// normal paths can't go to the last point because they always look forward a point
			// TODO: fix this ^
			lastPoint = current_point_idx == paths[current_path_idx].numPoints-1;
		} else {
			// holes don't look forward a point
			lastPoint = current_point_idx == paths[current_path_idx].numPoints;
		}

		// If we're at the end of the points, stop cutting so we can start the next path
		if (lastPoint) {
			Serial.println("Current path finished");
			stepperZ.moveTo(Conv*restHeight);
			path_started = false;
			current_point_idx = 0;
			current_path_idx++;

			// If we're done all paths then go back to design mode.
			if (current_path_idx == num_paths) {
				// Make sure tool is raised after path is finished
				while (stepperZ.distanceToGo() != 0) {
					stepperZ.run();
				}
				Serial.println("All paths finished");
				encoderDesignType();
			}
		}
		return true;
	} else {
		return false;
	}
}

void handleCutting() {
	// Start of cutting Logic
	Point goal = paths[current_path_idx].points[current_point_idx];
	Point next = paths[current_path_idx].points[current_point_idx + 1];

	// TODO: handle this case better
	if (goal.x == next.x && goal.y == next.y) {
		advance(goal, next, true);
		return;
	}

	// If we have not started the path, and the first point is behind us
	// keep the tool raised and return. We wait here until the first point
	// is in front of us and ready to be cut
	bool goal_behind = paths[current_path_idx].direction * signedDist(pose, goal) > 0;
	if (!path_started && goal_behind) {
		// Move tool closest to intersect with cutting path
		float desPos = desPosClosestToIntersect(pose, goal, next);
		
		stepperX.moveTo(Conv*desPos);

		// Update UI
		updateUI(desPos, goal, next);

		if (outputOn) outputSerial(pose, goal, stepperX.currentPosition()*1.0f/Conv, desPos, false);
		// outputSD(estPos[0], estPos[1], estYaw, goal, stepperX.currentPosition()*1.0f/Conv, desPos, false);
		debugging("path_started:%i, goal_behind:%i\n", (int)path_started, (int)goal_behind);
		if (debuggingOn) {
			debugging(goal, next);
		}
		return;
	}

	// If we get here start the path
	// TODO: change w.r.t. naming convention (i.e. cluster, pass, path, point)
	path_started = true;

	// Desired position if we intersect
	float desPos = desPosIntersect(pose, goal, next);
	float desZ = (matThickness == 0 && designType == FROM_FILE) ? (goal.z - minZ) : goal.z;			// if matThickness is set to 0 (drawing), then don't pierce!
	// Desired position if we do not intersect
	float desPosClosest = desPosClosestToIntersect(pose, goal, next);
	float desPosHole = desPosSimple(pose, goal);

	// Conditions for cutting
	bool handle_buttons_pressed = (digitalRead(BUTT_HANDLE_L) == LOW) && (digitalRead(BUTT_HANDLE_R) == LOW);
	bool handle_buttons_debounce = (millis() - timeLastDebounce) < debounceDelay;
	if (handle_buttons_pressed) { timeLastDebounce = millis(); }
	bool handle_buttons_ok = handle_buttons_pressed || handle_buttons_debounce;
	// bool handle_buttons_ok = (digitalRead(BUTT_HANDLE_L) == LOW && digitalRead(BUTT_HANDLE_R) == LOW) || 
	// 					   ((millis() - timeLastDebounce) < debounceDelay);
	// if (handle_buttons_ok) timeLastDebounce = millis();
	bool gantry_intersects = !isnan(desPos);
	bool goal_behind_router = paths[current_path_idx].direction * signedDist(pose, goal) > 0;
	bool gantry_angle_ok = angleFrom(goal, next) > (PI / 6);
	bool within_hole_tol = abs(signedDist(pose, goal)) < holeTolerance;

	// TODO: delete me
	bool newChecks[4] = {handle_buttons_ok, gantry_intersects, goal_behind_router, gantry_angle_ok};
	bool logged = false;
	for (int i=0; i<4; i++) {
		if (prevChecks[i] != newChecks[i] && !logged) {
		Serial.println("Checks have changed:");
		Serial.printf("\thandle_buttons_ok: %d\n", handle_buttons_ok);
		Serial.printf("\tgantry_intersects: %d\n", gantry_intersects);
		Serial.printf("\tgoal_behind_router: %d\n", goal_behind_router);
		Serial.printf("\tgantry_angle_ok: %d\n", gantry_angle_ok);
		logged = true;
		}
		prevChecks[i] = newChecks[i];
	}

	if (handle_buttons_ok && gantry_intersects && goal_behind_router && gantry_angle_ok && valid_sensors && paths[current_path_idx].feature == NORMAL) {
		// We are good to cut
		stepperZ.moveTo(Conv*desZ);
		stepperX.moveTo(Conv*desPos);

		// Path logging
		if (outputOn) outputSerial(pose, goal, stepperX.currentPosition()*1.0f/Conv, desPos, true);
		// outputSD(estPos[0], estPos[1], estYaw, goal, stepperX.currentPosition()*1.0f/Conv, desPos, true);

		// Evaluate whether to move on to next point
		advance(goal, next);
	} else {
		if (paths[current_path_idx].feature != HOLE) {
			// Stop cutting
			stepperZ.moveTo(Conv*restHeight);
			stepperX.moveTo(Conv*desPosClosest);
		} else {
			// Handle hole
			stepperZ.moveTo(Conv*restHeight);
			stepperX.moveTo(Conv*desPosHole);
			if (plungeReady && within_hole_tol) {
				plungeZ(desZ, holeFeedrate);
				advance(goal,next,true);
			} else {
				plungeReady = false;
			}
		}

		// Path logging
		if (outputOn) outputSerial(pose, goal, stepperX.currentPosition()*1.0f/Conv, desPosClosest, false);
		// outputSD(estPos[0], estPos[1], estYaw, goal, stepperX.currentPosition()*1.0f/Conv, desPosClosest, false);
	}

	// Update UI
	updateUI(desPos, goal, next);

	// Debugging
	if (debuggingOn) {
		debugging(goal, next);
	}
}