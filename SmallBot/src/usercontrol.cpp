#include "usercontrol.h"

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// This funciton will controll the bots drive motors based on controller input.
void driveM(){

  // Gets controller's anaolog stick valuus as pct and multiplies
  // them with 120 to get mv and stores that into variables.
  int a3 = Controller1.Axis3.position(pct) * 120;
  int a4 = Controller1.Axis4.position(pct) * 120;
  int a1 = Controller1.Axis1.position(pct) * 120;

  // The variables that were obtained above are now passed
  // on to movement functions for each of the drive motors.
  // More on how this function works is in the notebook. (Page X)
  frontRight.spin(fwd, a3 - a4 - a1, voltageUnits::mV);
  frontLeft.spin(fwd, -a3 - a4 - a1, voltageUnits::mV);
  backRight.spin(fwd, a3 + a4 - a1, voltageUnits::mV);
  backLeft.spin(fwd, -a3 + a4 - a1, voltageUnits::mV);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// This funciton will controll the bots drive motors based on controller input and orientation.
void driveMA(){

  // Gets controller's anaolog stick valuus as pct and multiplies
  // them with 120 to get mv and stores that into variables.
  int a3 = Controller1.Axis3.position(pct) * 120;
  int a4 = Controller1.Axis4.position(pct) * 120;
  int a1 = Controller1.Axis1.position(pct) * 120;

  // Angle is obtained from the gyro. 
  double currAngle = ((pi/180)*gyroM.heading());

  // Using the angle and voltages that were obtained above,
  // they will be passed on to movement functions for each of the drive motors.
  // More on how this function works is in the notebook. (Page X)
  frontLeft.spin(fwd, (a4*cos(flbrWheels-currAngle) + a3*sin(flbrWheels-currAngle)) - a1, voltageUnits::mV);
  frontRight.spin(fwd, -(a4*cos(frblWheels-currAngle) + a3*sin(frblWheels-currAngle)) - a1, voltageUnits::mV);
  backLeft.spin(fwd, (a4*cos(frblWheels-currAngle) + a3*sin(frblWheels-currAngle)) - a1, voltageUnits::mV);
  backRight.spin(fwd, -(a4*cos(flbrWheels-currAngle) + a3*sin(flbrWheels-currAngle)) - a1, voltageUnits::mV);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Controlls intake motors based on controller input.
void intakeM(){

  if(Controller1.ButtonR1.pressing()){ // If R1 is pressed the intake will spin towards the bot.
    leftIntake.spin(fwd, 100, percentUnits::pct);
    rightIntake.spin(fwd, 100, percentUnits::pct);
    //printf("Line %ld \n", ballDetector.value(pct));
  } else if(Controller1.ButtonR2.pressing()){ // Else if R2 is pressed the intake will spin away the bot.
    leftIntake.spin(fwd, -100, percentUnits::pct);
    rightIntake.spin(fwd, -100, percentUnits::pct);
    //printf("Line %ld \n", ballDetector.value(pct));
  } else { // If neither of those two are press the intakes will stop spinning.
    leftIntake.spin(fwd, 0, percentUnits::pct);
    rightIntake.spin(fwd, 0, percentUnits::pct);
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Testing/Debug tool
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