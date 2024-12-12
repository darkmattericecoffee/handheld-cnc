#ifndef LOGGING_H
#define LOGGING_H

#include "../types.h"

// Serial logging functions
void outputSerial(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting);
void debugging();

// SD card functions
bool initializeLogFile();
void outputSD(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting);
void closeSDFile();
void logPath();

#endif
