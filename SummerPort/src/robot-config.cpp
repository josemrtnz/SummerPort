#include <stdio.h>
#include "robot-config.h"

using namespace vex;

robotChasis::robotChasis( float wD, float tcL, float tcR, float tcB){
  wheelDiameter = wD;
  sL = tcL;
  sR = tcR;
  sS = tcB;
}

void robotChasis::set_drive_break_type(brakeType B){
  frontLeft.setBrake(B);
  frontRight.setBrake(B);
  backLeft.setBrake(B);
  backRight.setBrake(B);
}

void robotChasis::stopMotors(){
  frontLeft.stop();
  frontRight.stop();
  backLeft.stop();
  backRight.stop();
}

double robotChasis::getPI() { return PI; }
float robotChasis::get_flbr() { return flbrWheels; }
float robotChasis::get_frbl(){ return frblWheels; }
float robotChasis::getsL() { return sL; }
float robotChasis::getsR() { return sR; }
float robotChasis::getsS() { return sS; }
double robotChasis::getWheelCir(){ return PI * wheelDiameter; }

void vexcodeInit(robotChasis *simp) {
  simp->leftTracker.resetRotation();
  simp->rightTracker.resetRotation();
  simp->backTracker.resetRotation();

  wait(500, msec);
  // Gyro Callibrates
  simp->gyroM.calibrate();
  while(simp->gyroM.isCalibrating()){
    wait(50, msec);
  }

  // Prints the values of the tracking wheels in degrees.
  printf("%.0lf, %.0lf, %.0lf \n", simp->leftTracker.position(deg), simp->rightTracker.position(deg), simp->backTracker.position(deg));
}