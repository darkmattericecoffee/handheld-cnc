#include "logging.h"
#include "../config.h"
#include "../globals.h"
#include <Arduino.h>
#include "../math/geometry.h"

#define PACKET_START		0xAA
#define PACKET_END			0x55
// Constants for packet types
#define PACKET_SENSORS		0x01
#define PACKET_AUX			0x02
// Header/identifier constants
#define PACKET_HEADER		0xA0
#define PACKET_DESIGN_INFO	0xA1
#define PACKET_PATH			0xA2
#define PACKET_PATH_POINT	0xA3

char filename[MAX_STRING_LENGTH];

// Timing variables
static long unsigned timeLastOutput = 0;
static long unsigned timeLastOutputSD = 0;
static long unsigned timeLastDebug = 0;
static long unsigned timeLastClocked = 0;
static long unsigned timeLastFlush = 0;
static const long flushInterval = 1000;			// Interval to flush the SD card
static void* lastDebugCaller = nullptr;		// Store the address of the last debug call
static bool firstCall = true;				// Track if this is a new sequence of debug calls

// Read -----------------------------------------------------
void handleSerial() {
	if (Serial.available()) {
		char ch = Serial.read();
		switch (ch) {
			case 'o':
				outputSerialOn ^= 1;
				Serial.printf("Output to serial: %s\n", outputSerialOn ? "ON" : "OFF");
				break;
			case 's':
				stopwatchOn ^= 1;
				Serial.printf("Stopwatch: %s\n", stopwatchOn ? "ON" : "OFF");
				break;
			case 'l':
				outputSDOn ^= 1;
				Serial.printf("Output to SD: %s\n", outputSDOn ? "ON" : "OFF");
				break;
			case 'd':
				debuggingOn ^= 1;
				Serial.printf("Debugging: %s\n", debuggingOn ? "ON" : "OFF");
				break;
			case 'p':
				plottingOn ^= 1;
				Serial.printf("Plotting: %s\n", plottingOn ? "ON" : "OFF");
				break;
			default:
				break;
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
			logPath();

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
	snprintf(filename, MAX_STRING_LENGTH, "%s", sFilename.c_str());

	FsFile file;
	if (!file.open(filename, O_READ)) {
		return;
	}
	Serial.print("Filename: ");
	Serial.println(filename);

	// Reset paths
	for (int i = 0; i < MAX_PATHS; i++) {
		paths[i].feature = NORMAL;
		paths[i].numPoints = 0;
		
		// Reset all points in the path
		for (int j = 0; j < MAX_POINTS; j++) {
			paths[i].points[j] = {0.0f};
		}
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
				lastPoint = {0};
				currentPath->minZ = 0.0f;
				
				// Parse the M800 parameters
				char* ptr = line;
				while (*ptr) {
					// if (*ptr == 'D') currentPath->direction = atoi(ptr + 1);		// direction not needed anymore
					// if (*ptr == 'F') currentPath->feature = (Feature)atoi(ptr + 1);		// feauture not used
					// if (*ptr == 'A') currentPath->angle = atof(ptr + 1);
					ptr++;
				}
				num_paths++;
				Serial.printf("New Path%i!\n", currentPathIndex);
			}
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
			while (*ptr) {
				if (*ptr == 'G') {
					// TODO: make this cleaner and more universal
					if (atof(ptr+1) == 98) {
						currentPath->feature = DRILL;
					}
				}
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
					if (newPoint.z < currentPath->minZ) {
						currentPath->minZ = newPoint.z;			// TODO: maybe not necessary
						Serial.printf("Minimum z = %f", currentPath->minZ);
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
void outputSerial(Point goal, float toolPos, float desPos) {
	if(millis() - timeLastOutput >= dtOutput) {
		timeLastOutput = millis();

		// Calculate tool and desired positions
		float toolX = pose.x + toolPos*cosf(pose.yaw);
		float toolY = pose.y + toolPos*sinf(pose.yaw);

		float desX = pose.x + desPos*cosf(pose.yaw);
		float desY = pose.y + desPos*sinf(pose.yaw);

		Serial.printf(
			"POS:%f,%f,%f,%f,%f,%f,%f,%f,%f,%i,%f\n",
			pose.x,
			pose.y,
			pose.yaw,
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

void debugging(Point point1, Point point2) {
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
	Serial.printf("x:%f, y:%f, theta:%f, dir:%i\n", pose.x, pose.y, pose.yaw * 180.0 / PI, direction(point1, point2));
	Serial.printf("goal.x:%f,goal.y:%f,goal.z%f, next.x:%f,next.y:%f,next.z%f\n", point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);

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
		Serial.print("SD Log Time: ");
		Serial.println(SDLogTime);
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
		sprintf(filename, "logFiles/LOG%03d.bin", fileNumber++);
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

// Function to write header information
void writeFileHeader(const char* designName, uint16_t numPaths) {
	if (logFile) {
		FileHeader fileHeader;
		fileHeader.packetType = PACKET_HEADER;
		strncpy(fileHeader.firmwareVersion, FIRMWARE_VERSION, MAX_STRING_LENGTH);
		fileHeader.firmwareVersion[MAX_STRING_LENGTH-1] = '\0';			// Ensure null termination
		strncpy(fileHeader.designName, designName, MAX_STRING_LENGTH);
		fileHeader.designName[MAX_STRING_LENGTH-1] = '\0';				// Ensure null termination

		for (int i = 0; i < 4; i++) {
			fileHeader.calParams[i] = cal[i];
		}
		
		// TODO: add UNIX timestamp
		fileHeader.numPaths = numPaths;
		
		// Write the header to file
		logFile.write((uint8_t*)&fileHeader, sizeof(FileHeader));
		logFile.flush();

		Serial.printf("Header written to file: %s\n", fileHeader.designName);
	}
}

// Function to write path information
void writePathInfo(uint16_t pathIndex, uint8_t featureType) {
	if (logFile) {
		// uint8_t header = PACKET_START;
		// logFile.write(&header, 1);

		PathInfo pathInfo;
		pathInfo.packetType = PACKET_PATH;
		pathInfo.pathIndex = pathIndex;
		pathInfo.featureType = featureType;
		// pathInfo.numPoints = numPoints;
		// pathInfo.toolParams = toolParams;
		
		// Write the path info to file
		logFile.write((uint8_t*)&pathInfo, sizeof(PathInfo));

		// uint8_t footer = PACKET_END;
		// logFile.write(&footer, 1);
	}
}

// Function to write a single point in a path
void writePathPoint(uint16_t pathIndex, uint16_t pointIndex, Point point) {
	if (logFile) {
		// uint8_t header = PACKET_START;
		// logFile.write(&header, 1);

		PathPoint pathPoint;
		pathPoint.packetType = PACKET_PATH_POINT;
		pathPoint.pathIndex = pathIndex;
		pathPoint.pointIndex = pointIndex;
		pathPoint.x = point.x;
		pathPoint.y = point.y;
		pathPoint.z = point.z;
		
		// Write the point to file
		logFile.write((uint8_t*)&pathPoint, sizeof(PathPoint));

		// uint8_t footer = PACKET_END;
		// logFile.write(&footer, 1);
	}
}

// Write sensor data to SD card for datalogging
void writeSensorData(uint32_t time,  SensorData sensorArray[ns]) {
	if (logFile) {
		uint8_t header = PACKET_START;
		logFile.write(&header, 1);
		
		SensorsPacket packet;
		packet.packetType = PACKET_SENSORS;
		packet.time = time;
		
		// Copy all sensor data
		for (int i = 0; i < ns; i++) {
			packet.sensors[i] = sensorArray[i];
		}
		
		// Write the entire packet
		logFile.write((uint8_t*)&packet, sizeof(SensorsPacket));
		
		uint8_t footer = PACKET_END;
		logFile.write(&footer, 1);

		// Periodic flush
		if (millis() - timeLastFlush >= flushInterval) {
			logFile.flush();
			timeLastFlush = millis();
		}
	}
}

// Write auxilliary data to SD card for datalogging
void writeAuxData(Point goal, float toolPos, float desPos) {
	if (millis() - timeLastOutputSD >= dtOutputSD) {
		timeLastOutputSD = millis();
		
		if (logFile) {
			uint8_t header = PACKET_START;
			logFile.write(&header, 1);
			
			AuxPacket packet;
			packet.packetType = PACKET_AUX;
			packet.time = micros();  // Current time
			packet.pose = pose;
			packet.currPathIndex = current_path_idx;
			packet.currPointIndex = current_point_idx;
			packet.goal = goal;
			packet.toolPos = toolPos;
			packet.desPos = desPos;
			packet.cutState = cutState;
			
			logFile.write((uint8_t*)&packet, sizeof(AuxPacket));
			
			uint8_t footer = PACKET_END;
			logFile.write(&footer, 1);
			
			// Periodic flush
			if (millis() - timeLastFlush >= flushInterval) {
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
	// Log path data to SD card if enabled
	if (outputSDOn) {
		if (!initializeLogFile()) {
			Serial.println("Failed to initialize log file!");
			return;
		}

		if (designType == PRESET) {
			char designName[MAX_STRING_LENGTH];
			sprintf(designName, "preset_%i", designPreset);
			Serial.printf("Logging to preset design: %s\n", designName);
			writeFileHeader(designName, num_paths);
		} else {
			writeFileHeader(filename, num_paths);
		}
		

		for (int i = 0; i < num_paths; i++) {
			// Write path information
			writePathInfo(i, paths[i].feature);
			for (int j = 0; j < paths[i].numPoints; j++) {
				// Write each point in the path
				writePathPoint(i, j, paths[i].points[j]);
			}
		}

		if (logFile) logFile.flush();
	}

}