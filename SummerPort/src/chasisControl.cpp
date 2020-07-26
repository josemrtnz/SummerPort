#include "chasisControl.h"

////////////////////
// PID Settings
///////////

// x PID Settings
float xkP = 0; //1100
float xkD = 0; //20
float xkI = 0; //0

float xError = 0; // EnvoderValue - Desired Value (The further you are from your target the bigger this number will be)
float xPrevError = 0; // EncoderValue 20ms ago
float xDerivative = 0; // error - previousError : Speed
float xTotalError = 0; // totalError + error
int xCap = 2000;
///////////

// y PID Settings
float ykP = 0; //1100
float ykD = 0; //10
float ykI = 0; //0

float yError = 0; // EnvoderValue - Desired Value (The further you are from your target the bigger this number will be)
float yPrevError = 0; // EncoderValue 20ms ago
float yDerivative = 0; // error - previousError : Speed
float yTotalError = 0; // totalError + error
int yCap = 2000;
///////////

// Turn PID Settings
float turnkP = 280; //150
float turnkD = 0; //700
float turnkI = 0.75; //4

float turnError = 0; // EnvoderValue - Desired Value (The further you are from your target the bigger this number will be)
float turnPrevError = 0; // EncoderValue 20ms ago
float turnDerivative = 0; // error - previousError : Speed
float turnTotalError = 0; // totalError + error
int turnCap = 500;
///////////////////

///////////
// ~~~~~~~~~~~
////////////////////

////////////////////
// Robot Settings
///////////

float targetX = 0;
float targetY = 0;
float targetA = 0;

///////////
// ~~~~~~~~~~~
////////////////////

// movAb settings
double vMag;
double vectorD[2];
float xVoltage;
float yVoltage;
float angleVoltage;
///////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Controlls all motors in all directions
void moveDrive(double x, double y, float turn){
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
void coastMotor(){
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
void holdMotor(){
  frontLeft.setBrake(hold);
  frontRight.setBrake(hold);
  backLeft.setBrake(hold);
  backRight.setBrake(hold);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////

void stopMotors(){
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
int averageRPM(){
  return (fabs(frontRight.velocity(rpm)) + fabs(frontLeft.velocity(rpm)) + fabs(backRight.velocity(rpm)) + fabs(backLeft.velocity(rpm)))/4;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
double xPID(double inchTarget, double currentInch){
  xError = currentInch - inchTarget;
  xDerivative = xError - xPrevError;
  xTotalError += xError;

  if((xTotalError*xkI)>xCap) xTotalError = xCap/xkI;
  else if((xTotalError*xkI)<-xCap) xTotalError = -xCap/xkI;

  return -(xkP*xError + xkD*xDerivative + xkI*xTotalError);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
double yPID(double inchTarget, double currentInch){
  yError = currentInch - inchTarget;
  yDerivative = yError - yPrevError;
  yTotalError += yError;

  if((yTotalError*ykI)>yCap) yTotalError = yCap/ykI;
  else if((yTotalError*ykI)<-yCap) yTotalError = -yCap/ykI;

  return -(ykP*yError + ykD*yDerivative + ykI*yTotalError);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
double updateTurnPID(float angleTarget, float currentAngle){
  turnError = currentAngle - angleTarget;
  turnDerivative = turnError - turnPrevError;
  turnTotalError += turnError;

  if((turnTotalError*turnkI)>turnCap) turnTotalError = turnCap/turnkI;
  else if((turnTotalError*turnkI)<-turnCap) turnTotalError = -turnCap/turnkI;

  return (turnkP*turnError + turnkD*turnDerivative + turnkI*turnTotalError);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
int turningCap(double distanceMag){
  if(distanceMag>30.0) return 2000;
  else if(distanceMag<10.0) return 6000;
  else return (-200*distanceMag) + 8000;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void betterPID(){
  if(fabs(vectorD[0])>39.0){
    xkP = 1000;
    xkD = 1;
    xkI = 1.2; 
    xCap = 1100;
  }else if(fabs(vectorD[0])>30.0){
    xkP = 1000;
    xkD = 1;
    xkI = 0.8; 
    xCap = 1200;
  } else if(fabs(vectorD[0])>20.0){
    xkP = 1200;
    xkD = 1;
    xkI = 0.9; 
    xCap = 2000;
  } else if(fabs(vectorD[0])>10.0){
    xkP = 1300;
    xkD = 1;
    xkI = 0.7; 
    xCap = 2000;
  } else if(fabs(vectorD[0])>5.0){
    xkP = 1600;
    xkD = 1;
    xkI = 0.5; 
    xCap = 2000;
  } else {
    xkP = 3500;
    xkD = 1;
    xkI = 1.2; 
    xCap = 2000;
  }

  if(fabs(vectorD[1])>39.0){
    ykP = 1000;
    ykD = 1;
    ykI = 1.2;
    yCap = 1100;
  }else if(fabs(vectorD[1])>30.0){
    ykP = 1000;
    ykD = 1;
    ykI = .8;
    yCap = 1200;
  } else if(fabs(vectorD[1])>20.0){
    ykP = 1200;
    ykD = 1;
    ykI = 0.9;
    yCap = 2000;
  } else if(fabs(vectorD[1])>10.0){
    ykP = 1300;
    ykD = 1;
    ykI = 0.7;
    yCap = 2000;
  } else if(fabs(vectorD[1])>5.0){
    ykP = 1600;
    ykD = 1;
    ykI = 0.5;
    yCap = 2000;
  } else {
    ykP = 3500;
    ykD = 1;
    ykI = 1.2;
    yCap = 2000;
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void movAb(){

  vectorD[0] = targetX - xPos;
  vectorD[1] = targetY - yPos;
  vMag = sqrt((vectorD[0]*vectorD[0]) + (vectorD[1]*vectorD[1]));

  //////////////////////////////////////////////////
  // Caps go here

  int turnCap = turningCap(vMag);

  xVoltage = xPID(targetX, xPos);
  yVoltage = yPID(targetY, yPos);
  angleVoltage = updateTurnPID(targetA, angleD);

  if(angleVoltage>turnCap) angleVoltage = turnCap;
  else if(angleVoltage<-turnCap) angleVoltage = -turnCap;

  if(xVoltage>10000) xVoltage = 10000;
  else if(xVoltage<-10000) xVoltage = -10000;

  if(yVoltage>10000) yVoltage = 10000;
  else if(yVoltage<-10000) yVoltage = -10000;

  //////////////////////////////////////////////////

  moveDrive(xVoltage, yVoltage, -angleVoltage);  
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////

void arcTurn(float x, float y, float angleO, float arcRad){

}

//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void updateTargetPos(float x, float y, int angleO){
  vectorD[0] = x - xPos;
  vectorD[1] = y - yPos;
  betterPID();

  targetX = x;
  targetY = y;
  targetA = angleO;
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
int autoMain(){ // This task will control our robot's movement and intake during the autonmous period.

  coastMotor();

  while(true){
    movAb();
    task::sleep(20);
  }

  return 1;
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////