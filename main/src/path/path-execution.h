#ifndef PATH_EXECUTION_H
#define PATH_EXECUTION_H

#include "../types.h"

// Safety check functions
bool checkEndstops();

// Path advancement and execution functions
void advance(Point goal, Point next, bool autoAdvance);
void handleChickenHead();
void handleCutting();

#endif