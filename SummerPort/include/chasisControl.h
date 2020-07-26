#pragma once
#include "vex.h"

void moveDrive(double x, double y, float turn);
void updateTargetPos(float x, float y, int angleO);
int autoMain();

extern double vMag;
extern double vectorD[2];
extern float xVoltage;
extern float yVoltage;