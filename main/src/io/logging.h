#ifndef LOGGING_H
#define LOGGING_H

#include "../types.h"

// Serial logging functions
void outputSerial(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting);

// Debugging functions
void debugging(Point point1, Point point2);
void stopwatch();

// SD card functions
void updateFileList();
String getParentPath(const char* path);
void handleFileSelection();
bool validGCode(const char* gLine);
void parseGCodeFile(const String& sFilename);
bool initializeLogFile();
void outputSD(float estX, float estY, float estYaw, Point goal, float toolPos, float desPos, bool cutting);
void closeSDFile();
void logPath();

#endif
