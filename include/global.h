#pragma once
#include "main.h"

using namespace pros;

extern Motor topRight;
extern Motor topLeft;
extern Motor botRight;
extern Motor botLeft;

extern Motor leftI;
extern Motor rightI;

extern Motor lift;

extern Imu gyroM;
extern ADIAnalogIn liftPot;

extern Controller master;
extern float chasisAngle;

extern ADIEncoder leftTracker;
extern ADIEncoder rightTracker;
extern ADIEncoder backTracker;

extern float wheelRadius;
extern float sL;
extern float sR;
extern float sS;
