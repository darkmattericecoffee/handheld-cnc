#include "path-execution.h"
#include "../config.h"
#include "../globals.h"
#include "../motors/motors.h"
#include "../math/geometry.h"
#include "../io/logging.h"
#include "../ui/display.h"
#include "../ui/encoder.h"

bool prevChecks[4] = {false};

bool checkEndstops() {
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

void advance(Point goal, Point next, bool autoAdvance=false) {
	// Move through point indeces as needed
	if (direction(goal,next) * signedDist(pose,next) > 0 || autoAdvance) {
		// If next point is behind router, it becomes the new goal.
		current_point_idx++;
		// Serial.printf("On to point %i/%i\n", current_point_idx, paths[current_path_idx].numPoints);
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

				// Log data and close SD
				closeSDFile();

				Serial.println("All paths finished");
				state = RESET;
				encoderDesignType();
			}
		}
	}
}

void handleCutting() {
	// Start of cutting Logic
	Point goal = paths[current_path_idx].points[current_point_idx];
	Point next = paths[current_path_idx].points[current_point_idx + 1];

	// TODO: handle this case better
	if (goal.x == next.x && goal.y == next.y) {
		Serial.println("goal=next");
		advance(goal, next, true);
		return;
	}

	// Serial.printf("goal:%.2f,%.2f, next:%.2f,%.2f\n", goal.x, goal.y, next.x, next.y);

	// If we have not started the path, and the first point is behind us
	// keep the tool raised and return. We wait here until the first point
	// is in front of us and ready to be cut
	bool goal_behind_router = direction(goal, next) * signedDist(pose, goal) > 0;
	if (!path_started && goal_behind_router) {
		cutState = NOT_CUT_READY;
		// Move tool closest to intersect with cutting path
		float desPos = desPosClosestToIntersect(pose, goal, next);
		
		stepperX.moveTo(Conv*desPos);

		// Update UI
		updateUI(desPos, goal, next);

		if (outputSerialOn) outputSerial(goal, stepperX.currentPosition()*1.0f/Conv, desPos);
		if (outputSDOn) writeAuxData(goal, stepperX.currentPosition()*1.0f/Conv, desPos);
		if (debuggingOn) debugging(goal, next);
		// debugging("path_started:%i, goal_behind:%i\n", (int)path_started, (int)goal_behind);
		return;
	}

	// If we get here start the path
	path_started = true;

	// Desired position if we intersect
	float desPos = desPosIntersect(pose, goal, next);
	float desZ = (matThickness == 0 && designType == FROM_FILE) ? (goal.z - paths[current_path_idx].minZ) : goal.z;			// if matThickness is set to 0 (drawing), then don't pierce!
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
	bool gantry_angle_ok = angleFrom(goal, next) > (angleThreshold);		// TODO: how is this working?
	bool within_hole_tol = (abs(signedDist(pose, goal)) < holeTolerance) && (abs(desPosHole) < (xUsable/2));

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

	// TODO: handle valid_sensors better (want to prompt re-zeroing if sensors are bad)
	if (handle_buttons_ok && gantry_intersects && goal_behind_router && gantry_angle_ok && valid_sensors && paths[current_path_idx].feature == NORMAL) {
		cutState = CUTTING;

		actuate(desPos, desZ);

		// Evaluate whether to move on to next point
		advance(goal, next);
	} else if (paths[current_path_idx].feature == DRILL) {
		// Handle drilling		
		if (valid_sensors && plungeReady && within_hole_tol) {
			// TODO: add PLUNGE_READY into CutState
			if (cutState == NOT_CUT_READY) cutState = PLUNGING;
			stepperZ.setMaxSpeed(holeFeedrate);

			if (cutState == PLUNGING) {
				stepperZ.moveTo(Conv*desZ);
				if (stepperZ.distanceToGo() == 0) {
					cutState = RETRACTING;
				}
			} else if (cutState == RETRACTING) {
				stepperZ.moveTo(Conv*restHeight);
				if (stepperZ.distanceToGo() == 0) {
					stepperZ.setMaxSpeed(maxSpeedZ);
					plungeReady = false;
					advance(goal,next,true);
				}
			}
		} else {
			// Drill conditions are not met yet
			cutState = NOT_CUT_READY;
			plungeReady = false;		// TODO: make OO so that motor speed can be adjust automatically when flipped

			desPos = desPosHole;
			stepperZ.setMaxSpeed(maxSpeedZ);	// TODO: (see below)
			actuate(desPos, restHeight);
		}
	} else if (gantry_intersects && gantry_angle_ok && valid_sensors){
		// Conditions are good except for pressed buttons and the goal point being behind the router
		cutState = CUT_READY;
		
		// Retract
		desPos = desPosClosest;
		desZ = restHeight;
		actuate(desPos, desZ);
	} else {
		// None of the conditions are met!
		cutState = NOT_CUT_READY;
		
		// Retract
		desPos = desPosClosest;
		desZ = restHeight;
		actuate(desPos, desZ);
	}

	// Path logging
	if (outputSerialOn) outputSerial(goal, stepperX.currentPosition()*1.0f/Conv, desPos);
	if (outputSDOn)	writeAuxData(goal, stepperX.currentPosition()*1.0f/Conv, desPos);

	// Debugging
	if (debuggingOn) debugging(goal, next);

	// Update UI
	updateUI(desPos, goal, next);
}