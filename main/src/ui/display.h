#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "../types.h"

const int displayLines = 7;
const int centerLine = 3;

float exponentialSkew(float x);
void drawTypeMenu();
void drawShape();
void listFiles();
void updateFileList();
void handleFileSelection();
void drawCenteredText(const char* text, int size);
void drawFixedUI();
void drawUI(float desPosition, Point goal, Point next, uint8_t i);
void drawDirection();
void updateUI(float desPosition, Point goal, Point next);

#endif
