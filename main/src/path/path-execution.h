#ifndef PATH_EXECUTION_H
#define PATH_EXECUTION_H

#include "../types.h"

// Safety check functions
bool performSafetyChecks();

// Path advancement and execution functions
void advance(Point goal, Point next, bool autoAdvance);
void handleCutting();

#endif