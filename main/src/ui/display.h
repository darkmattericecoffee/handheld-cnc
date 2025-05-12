#ifndef DISPLAY_H
#define DISPLAY_H

#include "../types.h"
#include "../config.h"
#include "../globals.h"
#include "../math/geometry.h"

const int displayLines = 7;
const int centerLine = 3;

float exponentialSkew(float x);
void drawMenu(const char* options[], const int numOptions, int select);
void drawShape();
void listFiles();
void updateFileList();
void handleFileSelection();
void drawCenteredText(const char* text, int size);
void drawFixedUI();
void drawUI(Position desPosition, uint8_t i);
void updateUI(Position desPosition);

#endif
