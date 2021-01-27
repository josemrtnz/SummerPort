#pragma once

#include "robot-config.h"
#include "tracking.h"
#include <cmath>

class autonomousControl{
  public:
    void updateTargetPos(float x, float y, int angleDeg);
    void updateIntakePct(int pow);
    void updateFly(int rpm);
    void updateRoller(int pwr);
    void setPIDConstants(float flkP, float flkI, float flkD, int flCap,
                         float frkP, float frkI, float frkD, int frCap,
                         float blkP, float blkI, float blkD, int blCap,
                         float brkP, float brkI, float brkD, int brCap,
                         float rpmkP, float rpmkI, float rpmkD, int rpmCap,
                         float turnkP, float turnkI, float turnkD, int turnCap);
    void stopFly();

    void waitTilFull();
    void waitUntilBalls(int ball);
    void waitUntilDistance(float dis);
    void waitUntilSettled();

    void shootBall(int balls);
    void autoMain();
    autonomousControl(robotChasis *robot, odometry *tr);

  private:
    struct PIDSettings {
      float target; float curr; float error; float prevError;
            float derivative; float totalError;
      float kP, kI, kD;
      int cap;
    };

    PIDSettings flPID;
    PIDSettings frPID;
    PIDSettings blPID;
    PIDSettings brPID;
    PIDSettings turnPID;
    PIDSettings rpmPID;
    
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
    short ballsDeteced = 0;
    short ballsToShoot = 0;
    int lowerBound = 0;
    double vectorD[2];
    float vMag;
    double angleVoltage;
    float driveRatioV[4]; // fr fl br bl
    float driveRPM[4]; // fr fl br bl
    float currPos[4]; // x y angleR angleD
    float targetPos[3]; // x y angleD
    float driveVoltages[4]; // fr fl br bl
    

    void moveDrive(float fr, float fl, float br, float bl);
    float averageRPM();
    float updatePID(PIDSettings *good);
    int turnCap(float distanceMag);
    void movAB();
    void updateFlyTBH();
    void intakeMove();
    void flyMove();
    void rollerMove();
    void shootingBall();
    void driveRatio(float x, float y);
    void updateDriveRPM(float x, float y);
    void updateCurrPos();
    float findMaxRPM();
    void normalizeRPM(float max);
    void set_RPM_values();
};
