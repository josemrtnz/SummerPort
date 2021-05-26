#pragma once
#include "robot-config.h"
#include <math.h>
#include <cstdlib>

/// Class for user control.
///
/// This class will control the motors on the chasis using input from a controller.
/// Controller data is accessed via a pointer from a \ref robotChasis object.
class userControl{
  public:

    /// Runs the main drive loop.
    ///
    /// Will run a while loop that will check for user input and move the motors accordingly.
    /// @return NULL
    void driveLoop();

    /// Constructor for the class \ref userControl.
    ///
    /// This constructor will create an instance of the class \ref userControl.
    /// It will select starting driver mode based on the value of dM.
    /// @param *robot Pointer to an instance of \ref robotChasis object.
    /// @param dM Boolean that will run an absolute driver mode if set to false. 
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

