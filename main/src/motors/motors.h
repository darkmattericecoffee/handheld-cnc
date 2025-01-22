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
void plungeZ(float depth, float feedrate);

#endif
