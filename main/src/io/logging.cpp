#include "logging.h"

#define LINE_BUFFER_SIZE 100

// Timing variables
static long unsigned timeLastOutput = 0;
static long unsigned timeLastOutputSD = 0;
static long unsigned timeLastDebug = 0;
static long unsigned timeLastClocked = 0;
static long unsigned timeLastFlush = 0;
static void* lastDebugCaller = nullptr;  // Store the address of the last debug call
static bool firstCall = true;  // Track if this is a new sequence of debug calls

// Read -----------------------------------------------------
void handleSerial() {
	if (Serial.available()) {
		char ch = Serial.read();
		if (ch == 'd') {
			debuggingOn ^= 1;
		} else if (ch == 'p') {
			plottingOn ^= 1;
		}
	}
}

String getParentPath(const char* path) {
	String pathStr = String(path);
	int lastSlash = pathStr.lastIndexOf('/');
	
	if (lastSlash <= 0) {
		return "/";
	}
	
	return pathStr.substring(0, lastSlash);
}

void handleFileSelection() {
	if (current_file_idx >= 0 && current_file_idx < totalFiles) {
		String selectedFile = fileList[current_file_idx];
		
		// Check if it's a directory
		if (selectedFile.endsWith("/")) {
			// Remove trailing slash
			selectedFile = selectedFile.substring(0, selectedFile.length() - 1);

			char currentDirName[256];
			currentDir.getName(currentDirName, sizeof(currentDirName));
			
			// Handle special case for parent directory
			if (selectedFile == "..") {
				// Handle going up one directory
				currentDir.close();
				currentDir = sd.open(getParentPath(currentDirName));
			} else {
				// Handle entering a subdirectory
				currentDir.close();
				String newPath = String(currentDirName) + "/" + selectedFile;
				currentDir = sd.open(newPath);
			}
			
			current_file_idx = 0;
		} else {
			// Handle file selection
			parseGCodeFile(selectedFile);

			state = DESIGN_SELECTED;
			
			delay(100);
		}
	}
}

bool validCommand(const char* gLine) {
	// Returns true if the line contains a valid GCode command

	int numCommands = 3;
	const char* validCommands[numCommands] = {"G0", "G1", "G98"};
	int commandSizes[numCommands] = {2,2,3};

	for (int i = 0; i < numCommands; i++) {
		if (strncmp(gLine, validCommands[i], commandSizes[i]) == 0) {
			return true;
		}
	}

	return false;
}

bool validCoordinate(const char* gLine) {
	// Returns true if the line contains a valid coordinate (X, Y, or Z)

	int numCommands = 3;
	const char* validCommands[numCommands] = {"X", "Y", "Z"};
	int commandSizes[numCommands] = {1,1,1};

	for (int i = 0; i < numCommands; i++) {
		if (strncmp(gLine, validCommands[i], commandSizes[i]) == 0) {
			return true;
		}
	}

	return false;
}

void parseGCodeFile(const String& sFilename) {
	const char* filename = sFilename.c_str();

	FsFile file;
	if (!file.open(filename, O_READ)) {
		return;
	}
	Serial.print("Filename: ");
	Serial.println(filename);

	// Reset path
	// TODO: make the feature part of the point object instead of the path object
	// path.feature = NORMAL;
	path.numPoints = 0;
	
	// Reset all points in the path
	for (int j = 0; j < MAX_POINTS; j++) {
		path.points[j] = {0.0f};
		path.points[j].feature = NORMAL;
	}

	char line[LINE_BUFFER_SIZE];
	Path* activePath = &path;
	bool activeFeature = false;
	Point lastPoint = {0};

	while (file.fgets(line, sizeof(line))) {
		bool hasNewCoordinate = false;

		// Check for new path command
		if (strncmp(line, "M800", 4) == 0) {
			// TODO: handle different kinds of features
			// NOTE: M800 is unnecessary for the new version
			activeFeature = true;
			continue;
		}

		// Skip all the nonsense
		if (validCommand(line)) {
			activeFeature = true;
		} else if (!validCoordinate(line)){
			// once (X,Y,Z) coordinates have stopped being sent, the command is over
			activeFeature = false;
		}

		// Look for G moves
		if (activeFeature) {
			Point newPoint = lastPoint;
			char* ptr = line;
			
			// Parse X, Y, Z coordinates from line
			// TODO: needs some work
			while (*ptr) {
				switch (*ptr) {
				case 'G':
					// TODO: make this cleaner and more universal
					if (atof(ptr+1) == 98) newPoint.feature = DRILL;
					else if (atof(ptr+1) == 0) newPoint.feature = NORMAL;
					else if (atof(ptr+1) == 1) newPoint.feature = NORMAL;
					else if (atof(ptr+1) == 80) newPoint.feature = NORMAL; 		// G80 cancels current command (used in drill cycle)
					break;
				case 'X':
					newPoint.x = atof(ptr + 1);
					hasNewCoordinate = true;
					break;
				case 'Y':
					newPoint.y = atof(ptr + 1);
					hasNewCoordinate = true;
					break;
				case 'Z':
					newPoint.z = atof(ptr + 1);
					hasNewCoordinate = true;
					if (newPoint.z < activePath->minZ) {
						activePath->minZ = newPoint.z;			// TODO: maybe not necessary
						Serial.printf("Minimum z = %f", activePath->minZ);
					}
					if (newPoint.z > 4.0) {
						// TODO: this is bandaid for shitty gcode! Remove this
						newPoint.z = 4.0;
					}
					break;
				// TODO: parse feedrate (F) and, for holes, retract height (R)
				// case 'F':
				// 	newPoint.f = atof(ptr + 1);
				// 	break;
				// case 'R':
				}
				newPoint.f = feedrate;
				ptr++;
			}

			// Add point to current path if there's space
			if (hasNewCoordinate && activePath->numPoints < MAX_POINTS) {
				if (newPoint.feature != DRILL) {
					// Add a normal point
					activePath->points[activePath->numPoints] = newPoint;
					activePath->numPoints++;
					Serial.printf("Point: X(%f), Y(%f), Z(%f)\n", newPoint.x, newPoint.y, newPoint.z);
				} else if (activePath->numPoints + 2 < MAX_POINTS){
					// Make a drill cycle
					// TODO: make this cleaner and more universal for drill cycles
					float zVals[3] = {restHeight, newPoint.z, restHeight};
					for (int i = 0; i < 3; i++) {
						activePath->points[activePath->numPoints] = newPoint;
						activePath->points[activePath->numPoints].z = zVals[i];
						activePath->numPoints++;
						Serial.printf("Point: X(%f), Y(%f), Z(%f)\n", newPoint.x, newPoint.y, zVals[i]);
					}
				} else {
					Serial.println("Path is full!");
					break;
				}
			} else if (activePath->numPoints >= MAX_POINTS) {
				Serial.println("Path is full!");
				break;
			}

			lastPoint = newPoint;
		}
	}

	file.close();
}

// Write ------------------------------------------------------
void outputSerial(RouterPose rPose, Point goal, float toolPos, float desPos) {
	if(millis() - timeLastOutput >= dtOutput) {
		timeLastOutput = millis();

		// Calculate tool and desired positions
		float toolX = rPose.x + toolPos*cosf(rPose.yaw);
		float toolY = rPose.y + toolPos*sinf(rPose.yaw);

		float desX = rPose.x + desPos*cosf(rPose.yaw);
		float desY = rPose.y + desPos*sinf(rPose.yaw);

		Serial.printf(
			"POS:%f,%f,%f,%f,%f,%f,%f,%f,%f,%i,%f\n",
			rPose.x,
			rPose.y,
			rPose.yaw,
			goal.x,
			goal.y,
			toolX,
			toolY,
			desX,
			desY,
			cutState,
			toolPos
		);
	}
}

void debugging(Point point1, Position pos) {
	// TODO: make this sequential timing work better
	void* currentCaller = __builtin_return_address(0);
    if(millis() - timeLastDebug < dtDebug) {
        // If not enough time has passed, only allow a different caller once
        if(!firstCall || __builtin_return_address(0) == lastDebugCaller) {
            return;
        }
    } else {
        // Enough time has passed, reset the first call flag
        firstCall = true;
    }
    
    if(firstCall) {
        firstCall = false;  // Mark that we've done the first call in this sequence
    }
    
    lastDebugCaller = currentCaller;
	timeLastDebug = millis();

	// Print debug data
	Serial.printf("x:%f, y:%f, theta:%f\n", pose.x, pose.y, pose.yaw * 180.0 / PI);
	Serial.printf("idx:%i, goal.x:%f,goal.y:%f,goal.z:%f\n", current_point_idx, point1.x, point1.y, point1.z);
	Serial.printf("desPos.x:%f,desPos.y:%f,desPos.z:%f\n", pos.getX(), pos.getY(), pos.getZ());

	// Additional debug info can be uncommented as needed:
	/*
	Serial.printf("w0:%f,w0:%f,w0:%f,%w0:%f\n", 1000*estAngVel[0], 1000*estAngVel[1], 
					1000*estAngVel[2], 1000*estAngVel[3]);
	Serial.printf("x:%f,y:%f,theta:%f,xg:%f,yg:%f,desPos:%f",
					estPosX,estPosY,pose.yaw,goalX,goalY,desPos);
	Serial.printf("x_raw:%f,y_raw:%f\n",measVel[0][0],measVel[1][0]);
	Serial.printf("curr_pnt_idx:%i,curr_path_idx:%i\n",current_point_idx, current_path_idx);
	Serial.printf("Sensing time = %i\n", sensingTime);
	*/

}

// Variatic debug function (mimicking Serial.printf)
void debugging(const char* format, ...) {
	void* currentCaller = __builtin_return_address(0);
    if(millis() - timeLastDebug < dtDebug) {
        // If not enough time has passed, only allow a different caller once
        if(!firstCall || __builtin_return_address(0) == lastDebugCaller) {
            return;
        }
    } else {
        // Enough time has passed, reset the first call flag
        firstCall = true;
    }
    
    if(firstCall) {
        firstCall = false;  // Mark that we've done the first call in this sequence
    }

	lastDebugCaller = currentCaller;
	timeLastDebug = millis();

	va_list args;
	va_start(args, format);
	vdprintf((int)&Serial, format, args);
	va_end(args);
}

// Flash string version if needed
void debugging(const __FlashStringHelper* format, ...) {
	void* currentCaller = __builtin_return_address(0);
    if(millis() - timeLastDebug < dtDebug) {
        if(!firstCall || __builtin_return_address(0) == lastDebugCaller) {
            return;
        }
    } else {
        firstCall = true;
    }
    
    if(firstCall) {firstCall = false;}
    
    lastDebugCaller = currentCaller;
	timeLastDebug = millis();

	va_list args;
	va_start(args, format);
	vdprintf((int)&Serial, (const char*)format, args);
	va_end(args);
}

void stopwatch() {
	if(millis() - timeLastClocked >= dtStopwatch) {
		timeLastClocked = millis();

		Serial.println("=== Timing Information (microseconds) ===");
		Serial.print("Total Loop Time: ");
		Serial.println(totalLoopTime);
		Serial.print("Sensing Time: ");
		Serial.println(sensingTime_debug);
		Serial.print("Stepper Time: ");
		Serial.println(stepperTime);
		Serial.print("Serial Time: ");
		Serial.println(serialTime);
		Serial.print("Safety Check Time: ");
		Serial.println(safetyTime);
		Serial.print("Cutting Time: ");
		Serial.println(cuttingTime);
		Serial.println("=====================================");
	}
}

bool initializeLogFile() {
	// Close any existing file first
	if (logFile) {
		logFile.close();
	}
	
	// Create a new file with an incremental name
	char filename[20];
	int fileNumber = 0;
	
	// Find next available file number
	do {
		sprintf(filename, "logFiles/LOG%03d.txt", fileNumber++);
	} while (sd.exists(filename) && fileNumber < 1000);
	
	logFile = sd.open(filename, FILE_WRITE);
	if (!logFile) {
		Serial.println("Could not create log file!");
		return false;
	}
	
	logFile.flush();
	Serial.printf("Logging to file: %s\n", filename);
	return true;
}

void outputSD(RouterPose rPose, Point goal, float toolPos, float desPos) {
	if(millis() - timeLastOutputSD >= dtOutputSD) {
		timeLastOutputSD = millis();

		float toolX = rPose.x + toolPos*cosf(rPose.yaw);
		float toolY = rPose.y + toolPos*sinf(rPose.yaw);

		float desX = rPose.x + desPos*cosf(rPose.yaw);
		float desY = rPose.y + desPos*sinf(rPose.yaw);

		if (logFile) {
			// Write data in CSV format
			logFile.printf(
				"%f,%f,%f,%f,%f,%f,%f,%f,%f,%i,%f\n",
				rPose.x,
				rPose.y,
				rPose.yaw,
				goal.x,
				goal.y,
				toolX,
				toolY,
				desX,
				desY,
				cutState,
				toolPos
			);
			
			// Periodic flush to ensure data is written
			if (millis() - timeLastFlush >= 1000) {
				logFile.flush();
				timeLastFlush = millis();
			}
		}
	}
}

void closeSDFile() {
	if (logFile) {
		logFile.flush();
		logFile.close();
	}

	Serial.println("SD card file has been closed.");
}

void logPath() {
	// Log path data to serial if enabled
	if (outputOn) {
		for (int j = 0; j < path.numPoints; j++) {
			Serial.printf(
				"PATH:%f,%f,%f\n",
				path.points[j].x,
				path.points[j].y,
				path.points[j].z
			);
		}
	}

	// Log path data to SD card
	if (!initializeLogFile()) {
		return;
	}

	for (int j = 0; j < path.numPoints; j++) {
		logFile.printf(
			"PATH:%d,%f,%f,%f\n",
			path.points[j].x,
			path.points[j].y,
			path.points[j].z
		);
	}
}