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
  double xPos;
  double yPos;
  double angleR;
  double angleD;
  robotChasis *simp;
};