#include "chasisControl.h"

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
chasisControl::chasisControl(float of, float sc){
  offset = of;
  scalar = sc;

  xPID.kI = 0;
  xPID.kD = 0;
  xPID.kP = 0;
  xPID.derivative = 0;
  xPID.error = 0;
  xPID.prevError = 0;
  xPID.totalError = 0;
  xPID.cap = 2000;

  yPID.kI = 0;
  yPID.kD = 0;
  yPID.kP = 0;
  yPID.derivative = 0;
  yPID.error = 0;
  yPID.prevError = 0;
  yPID.totalError = 0;
  yPID.cap = 2000;

  turnPID.kI = 0.75;
  turnPID.kD = 0;
  turnPID.kP = 280;
  turnPID.derivative = 0;
  turnPID.error = 0;
  turnPID.prevError = 0;
  turnPID.totalError = 0;
  turnPID.cap = 500;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Controlls all motors in all directions
void chasisControl::moveDrive(double x, double y, float turn){
  frontLeft.spin( fwd, (x*cos(flbrWheels-angleR) + y*sin(flbrWheels-angleR)) + turn, voltageUnits::mV);
  frontRight.spin(fwd, -(x*cos(frblWheels-angleR) + y*sin(frblWheels-angleR)) + turn, voltageUnits::mV);
  backLeft.spin(fwd, (x*cos(frblWheels-angleR) + y*sin(frblWheels-angleR)) + turn, voltageUnits::mV);
  backRight.spin(fwd, -(x*cos(flbrWheels-angleR) + y*sin(flbrWheels-angleR)) + turn, voltageUnits::mV);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Sets each of the drive motors to the coast brake mode
void chasisControl::coastMotor(){
  frontLeft.setBrake(coast);
  frontRight.setBrake(coast);
  backLeft.setBrake(coast);
  backRight.setBrake(coast);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Sets each of the drive motors to the hold brake mode
void chasisControl::holdMotor(){
  frontLeft.setBrake(hold);
  frontRight.setBrake(hold);
  backLeft.setBrake(hold);
  backRight.setBrake(hold);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////

void chasisControl::stopMotors(){
  frontLeft.stop();
  frontRight.stop();
  backLeft.stop();
  backRight.stop();
}

//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Gets the average RPM of all the drive motors.
int chasisControl::averageRPM(){
  return (fabs(frontRight.velocity(rpm)) + fabs(frontLeft.velocity(rpm)) + fabs(backRight.velocity(rpm)) + fabs(backLeft.velocity(rpm)))/4;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
double chasisControl::updatePID(double targetG, double currentG, PIDsettings boi){
  boi.error = currentG - targetG;
  boi.derivative = boi.error - boi.prevError;
  boi.totalError += boi.error;

  if((boi.totalError*boi.kI)>boi.cap) boi.totalError = boi.cap/boi.kI;
  else if((boi.totalError*boi.kI)<-boi.cap) boi.totalError = -boi.cap/boi.kI;

  return -(boi.kP*boi.error + boi.kD*boi.derivative + boi.kI*boi.totalError);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
int chasisControl::turningCap(double distanceMag){
  if(distanceMag>30.0) return 2000;
  else if(distanceMag<10.0) return 6000;
  else return (-200*distanceMag) + 8000;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::betterPID(){
  if(fabs(vectorD[0])>39.0){
    xPID.kP = 1000;
    xPID.kD = 1;
    xPID.kI = 1.2; 
    xPID.cap = 1100;
  }else if(fabs(vectorD[0])>30.0){
    xPID.kP = 1000;
    xPID.kD = 1;
    xPID.kI = 0.8; 
    xPID.cap = 1200;
  } else if(fabs(vectorD[0])>20.0){
    xPID.kP = 1200;
    xPID.kD = 1;
    xPID.kI = 0.9; 
    xPID.cap = 2000;
  } else if(fabs(vectorD[0])>10.0){
    xPID.kP = 1300;
    xPID.kD = 1;
    xPID.kI = 0.7; 
    xPID.cap = 2000;
  } else if(fabs(vectorD[0])>5.0){
    xPID.kP = 1600;
    xPID.kD = 1;
    xPID.kI = 0.5; 
    xPID.cap = 2000;
  } else {
    xPID.kP = 3500;
    xPID.kD = 1;
    xPID.kI = 1.2; 
    xPID.cap = 2000;
  }

  if(fabs(vectorD[1])>39.0){
    yPID.kP = 1000;
    yPID.kD = 1;
    yPID.kI = 1.2;
    yPID.cap = 1100;
  }else if(fabs(vectorD[1])>30.0){
    yPID.kP = 1000;
    yPID.kD = 1;
    yPID.kI = .8;
    yPID.cap = 1200;
  } else if(fabs(vectorD[1])>20.0){
    yPID.kP = 1200;
    yPID.kD = 1;
    yPID.kI = 0.9;
    yPID.cap = 2000;
  } else if(fabs(vectorD[1])>10.0){
    yPID.kP = 1300;
    yPID.kD = 1;
    yPID.kI = 0.7;
    yPID.cap = 2000;
  } else if(fabs(vectorD[1])>5.0){
    yPID.kP = 1600;
    yPID.kD = 1;
    yPID.kI = 0.5;
    yPID.cap = 2000;
  } else {
    yPID.kP = 3500;
    yPID.kD = 1;
    yPID.kI = 1.2;
    yPID.cap = 2000;
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::movAb(){

  vectorD[0] = targetX - xPos;
  vectorD[1] = targetY - yPos;
  vMag = sqrt((vectorD[0]*vectorD[0]) + (vectorD[1]*vectorD[1]));

  //////////////////////////////////////////////////
  // Caps go here

  int turnCap = turningCap(vMag);

  xVoltage = updatePID(targetX, xPos, xPID);
  yVoltage = updatePID(targetY, yPos, yPID);
  angleVoltage = updatePID(targetA, angleD, turnPID);

  if(angleVoltage>turnCap) angleVoltage = turnCap;
  else if(angleVoltage<-turnCap) angleVoltage = -turnCap;

  if(xVoltage>10000) xVoltage = 10000;
  else if(xVoltage<-10000) xVoltage = -10000;

  if(yVoltage>10000) yVoltage = 10000;
  else if(yVoltage<-10000) yVoltage = -10000;

  //////////////////////////////////////////////////

  moveDrive(xVoltage, yVoltage, angleVoltage);  
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////

void arcTurn(float x, float y, float angleO, float arcRad){

}

//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::updateTargetPos(float x, float y, int angleO){
  vectorD[0] = x - xPos;
  vectorD[1] = y - yPos;
  betterPID();

  targetX = x;
  targetY = y;
  targetA = angleO;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::updateIntakePct(int pow){
  intakePct = pow;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::intakeMovePct(){
  leftIntake.spin(fwd, intakePct, pct);
  rightIntake.spin(fwd, intakePct, pct);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::waitUntilSettled(){
  wait(100, msec);
  while(averageRPM() != 0){
    wait(20, msec);
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::waitUntilDistance(float dis){
  wait(50, msec);
  while(dis < vMag){
    wait(20, msec);
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void chasisControl::waitUntilBalls(int ball){
  wait(50, msec);
  int ballsIntaken = 0;
  int32_t prevLine = 70;
  int32_t currLine = 0;
  while(ballsIntaken < ball){
    currLine = ballDetector.value(pct);

    if(prevLine > 65){
      if(ballDetector.value(pct)<65) ballsIntaken++;
    }

    prevLine = ballDetector.value(pct);
    wait(20, msec);
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
/*
void autoMain(){ // This task will control our robot's movement and intake during the autonmous period.

  coastMotor();

  while(true){
    movAb();
    intakeMovePct();
    task::sleep(20);
  }
}*/
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
