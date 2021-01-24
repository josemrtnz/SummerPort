#pragma once
#include "robot-config.h"
#include <math.h>
#include <stdio.h>

class odometry{
public:
  int updatePosition();
  int updateScreen();
  float getXPos();
  float getYPos();
  float getangleR();
  float getangleD();
  odometry(robotChasis *robot, float x, float y, float deg);

private:
  float xPos;
  float yPos;
  float angleR;
  float angleD;
  robotChasis *simp;
};