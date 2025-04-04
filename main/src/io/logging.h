#ifndef LOGGING_H
#define LOGGING_H

#include "../types.h"

// Struct definitions for your packets
struct SensorData {
	int16_t dx;
	int16_t dy;
	uint8_t sq;  // surface quality
};

struct SensorsPacket {
	uint8_t packetType;    // Set to PACKET_SENSORS (0x01)
	uint32_t time;         // microseconds since start
	SensorData sensors[4]; // Data for all 4 sensors
};

struct AuxPacket {
	uint8_t packetType;    // Set to PACKET_AUX (0x02)
	uint32_t time;         // microseconds since start
	RouterPose pose;
	uint16_t curr_path_idx;
	uint16_t curr_point_idx;
	Point goal;
	float toolPos;
	float desPos;
	int16_t cutState;
};

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
