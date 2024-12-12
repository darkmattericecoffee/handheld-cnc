#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "../types.h"

void drawShape();
void drawCenteredText(const char* text, int size);
void drawFixedUI();
void drawUI(float desPosition, Point goal, Point next, uint8_t i);
void drawDirection();
void updateUI(float desPosition, Point goal, Point next);

#endif
