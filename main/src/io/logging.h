#ifndef LOGGING_H
#define LOGGING_H

#include "../types.h"

// Serial logging functions
void handleSerial();
void outputSerial(RouterPose rPose, Point goal, float toolPos, float desPos);

// Debugging functions
void debugging(Point point1, Point point2);
void debugging(const char* format, ...);
void debugging(const __FlashStringHelper* format, ...);
void stopwatch();

// SD card functions
void updateFileList();
String getParentPath(const char* path);
void handleFileSelection();
bool validCommand(const char* gLine);
bool validCoordinate(const char* gLine);
void parseGCodeFile(const String& sFilename);
bool initializeLogFile();
void outputSD(RouterPose rPose, Point goal, float toolPos, float desPos);
void closeSDFile();
void logPath();

#endif
