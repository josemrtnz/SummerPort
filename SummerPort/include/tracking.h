#pragma once
#include "robot-config.h"
#include <math.h>
#include <stdio.h>

class odometry{
public:
  int updatePosition();
  int updateScreen();
  double getXPos();
  double getYPos();
  double getangleR();
  double getangleD();
  odometry(robotChasis *robot, double x, double y, double deg);

private:
  long double xPos;
  long double yPos;
  long double angleR;
  double angleD;
  robotChasis *simp;
};