#ifndef DISPLAY_H
#define DISPLAY_H

#include "../types.h"
#include "../config.h"
#include "../globals.h"
#include "../math/geometry.h"
#include "../actuation/actuator.h"
#include "path_preview.h"

const int displayLines = 7;
const int centerLine = 3;

float exponentialSkew(float x);
void drawIndicator(int16_t x, int16_t y, bool filled, uint16_t color);
void drawMenu(const char* options[], const int numOptions, int select);
void drawShape();
void listFiles();
void updateFileList();
void handleFileSelection();
void drawCenteredText(const char* text, int size);
void drawFixedUI();
void drawUI(Position desPosition, float progress, uint8_t i);
void updateUI(Position desPosition, float progress);
void resetPathPreview();

#endif
