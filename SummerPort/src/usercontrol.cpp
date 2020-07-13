#include "usercontrol.h"

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void driveX(){

  int a3 = Controller1.Axis3.position(pct) * 120;
  int a4 = Controller1.Axis4.position(pct) * 120;
  int a1 = Controller1.Axis1.position(pct) * 120;


  frontRight.spin(fwd, -a3 + a4 + a1, voltageUnits::mV);
  frontLeft.spin(fwd, a3 + a4 +a1, voltageUnits::mV);
  backRight.spin(fwd, -a3 - a4 +a1, voltageUnits::mV);
  backLeft.spin(fwd, a3 - a4 +a1, voltageUnits::mV);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void driveXA(){

  int a3 = Controller1.Axis3.position(pct) * 120;
  int a4 = Controller1.Axis4.position(pct) * 120;
  int a1 = Controller1.Axis1.position(pct) * 120;

  double currAngle = ((pi/180)*gyroM.heading());

  frontLeft.spin(fwd, (a4*cos(flbrWheels-currAngle) + a3*sin(flbrWheels-currAngle)) + a1, voltageUnits::mV);
  frontRight.spin(fwd, -(a4*cos(frblWheels-currAngle) + a3*sin(frblWheels-currAngle)) + a1, voltageUnits::mV);
  backLeft.spin(fwd, (a4*cos(frblWheels-currAngle) + a3*sin(frblWheels-currAngle)) + a1, voltageUnits::mV);
  backRight.spin(fwd, -(a4*cos(flbrWheels-currAngle) + a3*sin(flbrWheels-currAngle)) + a1, voltageUnits::mV);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
int * enterCoor(){
  static int coor[2];
  coor[0] = (int)xPos;
  coor[1] = (int)yPos;
  wait(100, msec);
  Controller1.Screen.clearScreen();
  wait(100, msec);
  while(true){
    if(Controller1.ButtonUp.pressing()) coor[1] += 1;
    if(Controller1.ButtonDown.pressing()) coor[1] -= 1;
    if(Controller1.ButtonRight.pressing()) coor[0] += 1;
    if(Controller1.ButtonLeft.pressing()) coor[0] -= 1;
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.print("x: %d y: %d     ", coor[0], coor[1]);
    
    if(Controller1.ButtonA.pressing()) break;
  }
  wait(100, msec);
  Controller1.Screen.clearScreen();
  wait(100, msec);
  return coor;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////