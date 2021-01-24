#pragma once
#include "v5.h"
#include "v5_vcs.h"
using namespace vex;

class robotChasis{
  public:
    double getPI();
    float getsL();
    float getsR();
    float getsS();
    float get_frbl();
    float get_flbr();
    double getWheelCir();

    brain Brain;
    competition Comp;
    controller Controller1;

    motor frontRight = motor(PORT16, ratio18_1, false);
    motor frontLeft = motor(PORT15, ratio18_1, false);
    motor backLeft = motor(PORT11, ratio18_1, false);
    motor backRight = motor(PORT12, ratio18_1, false);

    motor leftIntake = motor(PORT8, ratio6_1, true);
    motor rightIntake = motor(PORT20, ratio6_1, true);
    motor flyOuttake = motor(PORT7, ratio6_1, true);
    motor rollerIntake = motor(PORT9, ratio6_1, true);

    encoder leftTracker = encoder(Brain.ThreeWirePort.A);
    encoder rightTracker = encoder(Brain.ThreeWirePort.C);
    encoder backTracker = encoder(Brain.ThreeWirePort.E);
    limit shootD = limit(Brain.ThreeWirePort.H);

    inertial gyroM = inertial(PORT20);

    void set_drive_break_type(brakeType B);
    void stopMotors();
    robotChasis(float wD, float tcL, float tcR, float tcB);

  private:

    float wheelDiameter;
    double wheelCir;
    const double PI = 3.14159265359;

    float sL;
    float sR;
    float sS;

    float frblWheels = 2.35619449;
    float flbrWheels = 0.7853981634;

    int autonSelect;
};

void  vexcodeInit( robotChasis *simp );