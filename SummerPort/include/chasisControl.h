#pragma once

#include "robot-config.h"
#include "tracking.h"
#include <cmath>

/// Class for controlling the robot during the autonomous period.
///
/// A class for controlling the robot during the autonmous period of the competition.
/// This class has a loop called \ref autoMain will always be running.
/// This class uses a \ref robotChasis pointer to control the actuators and \ref odometry to read current location.
class autonomousControl{
  public:

    /// Updates the target position.
    ///
    /// This function will update the current target of the robot with x coordinates, y coordinates, and orientation.
    /// @param x X in inches.
    /// @param y Y in inches.
    /// @param angleDeg Orientation in degrees.
    void updateTargetPos(float x, float y, int angleDeg);

    /// Updates the target power for the intake rollers.
    ///
    /// This function will update the current power target for right and left intakes.
    /// @param pow Power in percentage units.
    void updateIntakePct(int pow);

    /// Updates the target rpm for the flywheel.
    ///
    /// This function will update the current rpm target for the flywheel.
    /// @param rpm RPM in rotation per minute units.
    void updateFly(int rpm);

    /// Updates the target power for the mid rollers.
    ///
    /// This function will update the current power target for the mid rollers.
    /// @param pow Power in percentage units.
    void updateRoller(int pwr);

    /// Waits until the robot is full.
    ///
    /// This function has a while loop that will run until it detects a ball at the internal limit switch.
    void waitTilFull();

    /// Waits until the robot has picked up a number of balls.
    ///
    /// Waits until the robot detects that a number of specified balls has passed through the internal limit switch.
    /// @param ball Number of balls to wait for.
    void waitUntilBalls(int ball);

    /// Waits until the robot is within a distance of the target.
    ///
    /// Waits until the robot is within a specified distance of the target position.
    /// @param dis Distance to the target in inches.
    void waitUntilDistance(float dis);

    /// Waits until the robot stops moving.
    ///
    /// Waits until the rpm of the drive motors falls below 2.
    void waitUntilSettled();

    /// Waits until the robot's orientation is within the target.
    ///
    /// Waits until the robot's orrientation is within the specified angular distance of the target orientation.
    /// @param deg Degrees to wait until it reaches the target.
    void waitUntilDeg(float deg);

    /// Shoots specified amount of balls.
    ///
    /// Will spin the flywheel and midrollers until the specified amount of balls are shot.
    /// @param balls Number of balls to shoot.
    void shootBall(int balls);

    /// Main loop for the autonomous period.
    ///
    /// This function has a while loop that will always be running during the autonmous period.
    /// The while loop will call functions to move the drive to the target position and any other motors will be set to their target. 
    void autoMain();

    /// N/A
    /// 
    /// N/A
    void visionTowerAlign(int angDeg);

    /// Stops Flywheel
    ///
    /// This function will stop the flywheel.
    void stopFly();

    /// Sets PID constants for the robot in autonomous control.
    ///
    /// This function will set the PID settings for the x, y, and turning.
    /// @param xkP x kP Constant
    /// @param xkI x kI Constant
    /// @param xkD x kD Constant
    /// @param xCap x cap setting for the total error
    /// @param ykP y kP Constant
    /// @param ykI y kI Constant
    /// @param ykD y kD Constant
    /// @param yCap y cap setting for the total error
    /// @param turnkP turn kP Constant
    /// @param turnkI turn kI Constant
    /// @param turnkD turn kD Constant
    /// @param turnCap turn cap setting for the total error
    
    void setPIDConstants(float xkP, float xkI, float xkD, int xCap,
                         float ykP, float ykI, float ykD, int yCap,
                         float turnkP, float turnkI, float turnkD, int turnCap);

    /// Constructor for the class \ref autonomousControl.
    ///
    /// This constructor will create an instance of the class \ref autonomousControl.
    /// It will save a pointer to an \ref robotChasis object for acutating the motors.
    /// It will save a pointer to an \ref odometry object for reading postion and feedback controls.
    /// @param *robot Pointer of an \ref robotChasis object.
    /// @param *tr Pointer of an \ref odometry object.
    autonomousControl(robotChasis *robot, odometry *tr);

  private:
    struct PIDSettings {
      float target; float curr; float error; float prevError;
            float derivative; float totalError;
      float kP, kI, kD;
      int cap;
    };

    PIDSettings xPID;
    PIDSettings yPID;
    PIDSettings turnPID;
    
    robotChasis *simp;
    odometry *tracking;

    int intakePct = 0;
    int flyWheelRPM = 0;
    int rollerPct = 0;
    float flykI = .4;
    double flyError = 0;
    double flyVoltage = 0;
    double flyLastError = 0;
    int flyApprox = 0;
    bool firstCross = true;
    double flyZero = 0;
    bool shooting = false;
    bool prevShot = 0;
    double rightEncoder;
    double leftEncoder;
    double backEncoder;
    int visionStatus = 0;
    bool movAB_Enabled = true;
    short ballsDeteced = 0;
    short ballsToShoot = 0;
    int lowerBound = 0;
    double vectorD[2];
    float vMag;
    double angleVoltage;
    

    void moveDrive(float x, float y, float turn);
    void odometryMove(bool oMove);
    void turnVision();
    void forwardVision();
    void strafeVision();
    float averageRPM();
    float updatePID(PIDSettings *good);
    int turnCap(float distanceMag);
    void movAB();
    void updateFlyTBH();
    void intakeMove();
    void flyMove();
    void rollerMove();
    void shootingBall();
    void updateCurrPos();
    void updateVisionPos();
    void moveVision();
    void driveM(double a3, double a4, double a1);
};
