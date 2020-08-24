#pragma once
#include "vex.h"
#include "spline.h"

//void moveDrive(double x, double y, float turn);
void updateTargetPos(float x, float y, int angleO);
int autoMain();
void waitUntilSettled();
void waitUntilDistance(float dis);
void updateIntakePct(int pow);
void waitUntilBalls(int ball);
void betterPID();
void splineDriveSet(std::vector<double> X, std::vector<double> Y, int angleO, bool right);

extern double vMag;
extern double vectorD[2];
extern float xVoltage;
extern float yVoltage;