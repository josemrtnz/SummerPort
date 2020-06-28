#include "chassisControl.h"

////////////////////
// PID Settings
///////////

// Distance PID Settings
double distancekP = 700; //150
double distancekD = 100; //700
double distancekI = 0; //4

int distanceError = 0; // EnvoderValue - Desired Value (The further you are from your target the bigger this number will be)
int distancePrevError = 0; // EncoderValue 20ms ago
int distanceDerivative = 0; // error - previousError : Speed
int distanceTotalError = 0; // totalError + error
///////////

// Turn PID Settings

///////////////////


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Controlls right side motors with voltage
void moveRight(int mVolts){

  // Applies a voltage to both right hand side motors.
  topRight.move_voltage(mVolts);
  botRight.move_voltage(mVolts);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Controlls left side motors with voltage
void moveLeft(int mVolts){

  // Applies a voltage to both left hand side motors.
  topLeft.move_voltage(mVolts);
  botLeft.move_voltage(mVolts);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Sets each of the drive motors to the coast brake mode
void coastMotor(){
  topLeft.set_brake_mode(E_MOTOR_BRAKE_COAST);
  topRight.set_brake_mode(E_MOTOR_BRAKE_COAST);
  botLeft.set_brake_mode(E_MOTOR_BRAKE_COAST);
  botRight.set_brake_mode(E_MOTOR_BRAKE_COAST);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// Sets each of the drive motors to the hold brake mode
void holdMotor(){
  topLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  topRight.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  botLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  botRight.set_brake_mode(E_MOTOR_BRAKE_HOLD);
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void moveDistancePID(int inchTarget){

  // Sets the drive motors to coast
  coastMotor();

  // Converts inches to degrees
  double inchtoDeg = (inchTarget/12.566) * 360;

  // Boolean status for the while loop
  bool isRunning = true;

  //Get average encoder values of the 4 motors.
  double averageEncoder = (topRight.get_position() + topLeft.get_position()
                          + botLeft.get_position() + botRight.get_position())/4;

  //Gets target degrees for the the motor to spin towards.
  double targetDeg = averageEncoder + inchtoDeg;

  while(isRunning){

    //Get average encoder values of the 4 motors.
    averageEncoder = averageEncoder = (topRight.get_position() + topLeft.get_position()
                                      + botLeft.get_position() + botRight.get_position())/4;

    //Calculates error
    distanceError = averageEncoder - targetDeg;

    //Calculate derivative
    distanceDerivative = distanceError - distancePrevError;

    //Calculate integral only if within 2 inches of target distance
    if(abs(distanceError)<30) distanceTotalError += distanceError;

    //Calvulates current volatge needed to move the motors
    double motorVoltage = (distanceError * distancekP) + (distanceTotalError * distancekI) + (distanceDerivative * distancekD);

    //Applies the motor volatge to the drive motors
    moveLeft(-motorVoltage);
    moveRight(-motorVoltage);

    //Previous error
    distancePrevError = distanceError;

    //Exit condition
    if((fabs(botRight.get_actual_velocity())<1) && (abs(distanceError)<1)){
       moveLeft(0);
       moveRight(0);
       isRunning = false;
    }
    delay(20);
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int updatePID(float inchTarget, float currentInch){
  distanceError = currentInch - inchTarget;
  distanceDerivative = distanceError - distancePrevError;
  distanceTotalError += distanceError;

  return -(distancekP*distanceError + distancekD*distanceDerivative + distancekI+distanceTotalError);
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
void movAb(float x, float y){
  float mVoltage;
  float magnitude;
  double vectorD[2];
  float angleTarget;
  coastMotor();

  while(true){
    vectorD[0] = x - getxPos();
    vectorD[1] = y - getyPos();
    magnitude = sqrt(powf(vectorD[0], 2) + powf(vectorD[1], 2));
    mVoltage = updatePID(magnitude, 0);
    angleTarget = atan2(vectorD[0], vectorD[1]);
    mVoltage = 0;
    lcd::print(3, "%lf", angleTarget * (180/pit));
    moveLeft(mVoltage - 5000*(getAngle() + angleTarget));
    moveRight(mVoltage + 5000*(getAngle() + angleTarget));
    delay(20);
  }
}
//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
