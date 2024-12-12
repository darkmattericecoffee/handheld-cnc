#ifndef PATH_GENERATORS_H
#define PATH_GENERATORS_H

void lineGenerator();
void sinGenerator();
void zigZagGenerator();
void doubleLineGenerator();
void circleGenerator();
void squareGenerator();
void squareGeneratorSine();
void squareGeneratorWave();
void makePath();
void parseNC(const char* filename);

// Path properties
extern const float sinAmp;
extern const float sinPeriod;
extern const float pathMax_y;
extern const float circleDiameter;

#endif
