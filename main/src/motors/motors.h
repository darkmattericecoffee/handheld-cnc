#ifndef MOTORS_H
#define MOTORS_H
#include "types.h"

void motorSetup();
void driverSetup();
void enableStepperZ();
void enableStepperRL();
void disableStepperZ();
void disableStepperRL();
void stopStepperX();
void stopStepperZ();
void machineZeroXY();
void workspaceZeroZ();
void workspaceZeroXY();
void actuate(Position pos);

#endif
