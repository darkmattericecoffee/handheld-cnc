#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

void sensorSetup();
void doSensing();
void doSensingLinear();
void sensorPlotting();
void readEepromCalibration();
void writeEepromCalibration();
void calibrate();
int16_t convTwosComp(int16_t value);

#endif
