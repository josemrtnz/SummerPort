#pragma once
#include "robot-config.h"
#include <math.h>

class userControl{
  public:
    void driveLoop();
    userControl(robotChasis *robot, bool dM);

  private:
    robotChasis *simp;
    bool driverMode;
    bool flyWheelOn = false;
    bool flyLastPress = false;
    int a3, a4, a1;
    int flyWheelPow;
    bool flySpeedToggle = false;
    bool flyFast = true;

    void setDriveMode();
    void driveM();
    void driveMA();
    void intakeM();
    void storageRoller();
    void flyWheelToggle();
    void setBrakeMode();
};

