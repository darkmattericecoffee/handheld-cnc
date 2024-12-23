#include "logging.h"
#include "../config.h"
#include "../globals.h"
#include <Arduino.h>

// Timing variables
static long unsigned timeLastOutput = 0;
static long unsigned timeLastOutputSD = 0;
static long unsigned timeLastDebug = 0;
static long unsigned timeLastFlush = 0;

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

void debugging() {
	if(millis() - timeLastDebug >= dtDebug) {
		timeLastDebug = millis();

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

		// Print debug data
		Serial.printf("x:%f,y:%f,theta:%f\n", estPos[0], estPos[1], estYaw * 180.0 / PI);

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

bool initializeLogFile() {
	// Close any existing file first
	if (dataFile) {
		dataFile.close();
	}
	
	// Create a new file with an incremental name
	char filename[20];
	int fileNumber = 0;
	
	// Find next available file number
	do {
		sprintf(filename, "LOG%03d.txt", fileNumber++);
	} while (sd.exists(filename) && fileNumber < 1000);
	
	dataFile = sd.open(filename, FILE_WRITE);
	if (!dataFile) {
		Serial.println("Could not create log file!");
		return false;
	}
	
	dataFile.flush();
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

		if (dataFile) {
			// Write data in CSV format
			dataFile.printf(
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
				dataFile.flush();
				timeLastFlush = millis();
			}
		}
	}
}

void closeSDFile() {
	if (dataFile) {
		dataFile.flush();
		dataFile.close();
	}

	Serial.println("SD card file has been closed.");
}

void logPath() {
	// Log path data to serial if enabled
	if (outputMode) {
		for (int i = 0; i < num_paths; i++) {
			for (int j = 0; j < num_points; j++) {
				Serial.printf(
					"PATH:%d,%f,%f,%f\n",
					i,
					paths[i][j].x,
					paths[i][j].y,
					paths[i][j].z
				);
			}
		}
	}

	// Log path data to SD card
	if (!initializeLogFile()) {
		return;
	}

	for (int i = 0; i < num_paths; i++) {
		for (int j = 0; j < num_points; j++) {
			dataFile.printf(
				"PATH:%d,%f,%f,%f\n",
				i,
				paths[i][j].x,
				paths[i][j].y,
				paths[i][j].z
			);
		}
	}
}