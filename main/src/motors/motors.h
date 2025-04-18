#ifndef MOTORS_H
#define MOTORS_H

void motorSetup();
void driverSetup();
void enableStepperZ();
void disableStepperZ();
void stopStepperX();
void stopStepperZ();
void machineZeroX();
void workspaceZeroZ();
void workspaceZeroXY();
void actuate(float desX, float desZ);

#endif
