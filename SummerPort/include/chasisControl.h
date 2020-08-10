#pragma once
#include "vex.h"

struct PIDsettings{
  float kP;
  float kD;
  float kI;
  float error;
  float prevError;
  float derivative;
  float totalError;
  int cap;
};

class chasisControl{
  public:
    chasisControl(float offset, float scalar);
    chasisControl();
    void updateTargetPos(float x, float y, int angleO);
    void waitUntilSettled();
    void waitUntilDistance(float dis);
    void updateIntakePct(int pow);
    void waitUntilBalls(int ball);
    void betterPID();
    double getvMag();
    void coastMotor();
    void movAb();
    void intakeMovePct();
    double vMag;

  private:
    float offset;
    float scalar;
    double vectorD[2];
    float xVoltage;
    float yVoltage;
    float angleVoltage;
    PIDsettings xPID;
    PIDsettings yPID;
    PIDsettings turnPID;
    float targetX = 0;
    float targetY = 0;
    float targetA = 0;
    int intakePct = 0;

    void moveDrive(double x, double y, float turn);
    void holdMotor();
    void stopMotors();
    int averageRPM();
    int turningCap(double distanceMag);
    double updatePID(double targetG, double currentG, PIDsettings boi);
};
