#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

void sensorSetup();
void doSensing();
void doSensingLinear();
void sensorPlotting();
void readEepromCalibration(float (&cVal)[2][4]);
void calibrate();
int16_t convTwosComp(int16_t value);

#endif
