#include "logging.h"
#include "../config.h"
#include "../globals.h"
#include <Arduino.h>

#define LINE_BUFFER_SIZE 100

// Timing variables
static long unsigned timeLastOutput = 0;
static long unsigned timeLastOutputSD = 0;
static long unsigned timeLastDebug = 0;
static long unsigned timeLastClocked = 0;
static long unsigned timeLastFlush = 0;

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

bool validGCode(const char* gLine) {
	int numCommands = 6;
	const char* validCommands[numCommands] = {"G0", "G1", "G98", "X", "Y", "Z"};
	int commandSizes[numCommands] = {2,2,3,1,1,1};

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

	int currentPathIndex = -1;
	char line[LINE_BUFFER_SIZE];
	Path* currentPath = &paths[0];
	bool activeFeature = false;
	Point lastPoint = {0};
	num_paths = 0;

	while (file.fgets(line, sizeof(line))) {
		bool hasNewCoordinate = false;

		// Check for new path command
		if (strncmp(line, "M800", 4) == 0) {
			if (currentPathIndex < MAX_PATHS) {
				activeFeature = true;
				currentPathIndex++;
				currentPath = &paths[currentPathIndex];
				currentPath->direction = 1;  // Default values
				currentPath->feature = NORMAL;
				// currentPath->angle = 0.0f;			// TODO!
				currentPath->numPoints = 0;
				lastPoint = {0};
				minZ = 0.0f;
				
				// Parse the M800 parameters
				char* ptr = line;
				while (*ptr) {
					if (*ptr == 'D') currentPath->direction = atoi(ptr + 1);
					if (*ptr == 'F') currentPath->feature = (Feature)atoi(ptr + 1);
					// if (*ptr == 'A') currentPath->angle = atof(ptr + 1);
					ptr++;
				}
				num_paths++;
				Serial.println("New Path!");
			}
			continue;
		}

		// Skip all the nonsense
		// if (!activeFeature) continue;

		// Look for G moves
		if (validGCode(line)) {
			Point newPoint = lastPoint;
			char* ptr = line;
			
			// Parse X, Y, Z coordinates from line
			while (*ptr) {
				if (*ptr == 'X') {
					newPoint.x = atof(ptr + 1);
					hasNewCoordinate = true;
				}
				if (*ptr == 'Y') {
					newPoint.y = atof(ptr + 1);
					hasNewCoordinate = true;
				}
				if (*ptr == 'Z') {
					newPoint.z = atof(ptr + 1);
					hasNewCoordinate = true;
					if (newPoint.z < minZ) {
						minZ = newPoint.z;
					}
				}
				// TODO: For holes - parse feedrate (F) and retract height (R)
				ptr++;
			}

			// Add point to current path if there's space
			if (hasNewCoordinate && currentPath->numPoints < MAX_POINTS) {
				currentPath->points[currentPath->numPoints] = newPoint;
				currentPath->numPoints++;
				Serial.printf("Point: X(%f), Y(%f), Z(%f)\n", newPoint.x, newPoint.y, newPoint.z);
			}

			lastPoint = newPoint;
		}
	}

	file.close();
}

// Write ------------------------------------------------------
void outputSerial(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting) {
	if(millis() - timeLastOutput >= dtOutput) {
		timeLastOutput = millis();

		// Calculate tool and desired positions
		float toolX = estX + toolPos*cosf(estYaw);
		float toolY = estY + toolPos*sinf(estYaw);

		float desX = estX + desPos*cosf(estYaw);
		float desY = estY + desPos*sinf(estYaw);

		Serial.printf(
			"POS:%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f\n",
			estX,
			estY,
			estYaw,
			goal.x,
			goal.y,
			toolX,
			toolY,
			desX,
			desY,
			cutting,
			toolPos
		);
	}
}

void debugging(Point point1, Point point2) {
	if(millis() - timeLastDebug >= dtDebug) {
		timeLastDebug = millis();

		// Print debug data
		Serial.printf("x:%f,y:%f,theta:%f\n", estPos[0], estPos[1], estYaw * 180.0 / PI);
		Serial.printf("goal.x:%f,goal.y:%f,next.x:%f,next.y:%f\n", point1.x, point1.y, point2.x, point2.y);

		// Additional debug info can be uncommented as needed:
		/*
		Serial.printf("w0:%f,w0:%f,w0:%f,%w0:%f\n", 1000*estAngVel[0], 1000*estAngVel[1], 
					 1000*estAngVel[2], 1000*estAngVel[3]);
		Serial.printf("x:%f,y:%f,theta:%f,xg:%f,yg:%f,desPos:%f",
					 estPosX,estPosY,estYaw,goalX,goalY,desPos);
		Serial.printf("x_raw:%f,y_raw:%f\n",measVel[0][0],measVel[1][0]);
		Serial.printf("curr_pnt_idx:%i,curr_path_idx:%i\n",current_point_idx, current_path_idx);
		Serial.printf("Sensing time = %i\n", sensingTime);
		*/
	}
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

void outputSD(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting) {
	if(millis() - timeLastOutputSD >= dtOutputSD) {
		timeLastOutputSD = millis();

		float toolX = estX + toolPos*cosf(estYaw);
		float toolY = estY + toolPos*sinf(estYaw);

		float desX = estX + desPos*cosf(estYaw);
		float desY = estY + desPos*sinf(estYaw);

		if (logFile) {
			// Write data in CSV format
			logFile.printf(
				"%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f\n",
				estX,
				estY,
				estYaw,
				goal.x,
				goal.y,
				toolX,
				toolY,
				desX,
				desY,
				cutting,
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
		for (int i = 0; i < num_paths; i++) {
			for (int j = 0; j < paths[current_path_idx].numPoints; j++) {
				Serial.printf(
					"PATH:%d,%f,%f,%f\n",
					i,
					paths[i].points[j].x,
					paths[i].points[j].y,
					paths[i].points[j].z
				);
			}
		}
	}

	// Log path data to SD card
	if (!initializeLogFile()) {
		return;
	}

	for (int i = 0; i < num_paths; i++) {
		for (int j = 0; j < paths[current_path_idx].numPoints; j++) {
			logFile.printf(
				"PATH:%d,%f,%f,%f\n",
				i,
				paths[i].points[j].x,
				paths[i].points[j].y,
				paths[i].points[j].z
			);
		}
	}
}