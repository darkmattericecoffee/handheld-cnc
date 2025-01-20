#ifndef LOGGING_H
#define LOGGING_H

#include "../types.h"

// Serial logging functions
void handleSerial();
void outputSerial(RouterPose rPose, Point goal, float toolPos, float desPos, bool cutting);

// Debugging functions
void debugging(Point point1, Point point2);
void stopwatch();

// SD card functions
void updateFileList();
String getParentPath(const char* path);
void handleFileSelection();
bool validCommand(const char* gLine);
bool validCoordinate(const char* gLine);
void parseGCodeFile(const String& sFilename);
bool initializeLogFile();
void outputSD(RouterPose rPose, Point goal, float toolPos, float desPos, bool cutting);
void closeSDFile();
void logPath();

#endif
