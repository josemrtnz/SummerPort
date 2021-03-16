#include "chasisControl.h"

autonomousControl::autonomousControl(robotChasis *robot, odometry *tr){
  simp = robot;
  tracking = tr;
}

void autonomousControl::setPIDConstants(float xkP, float xkI, float xkD, int xCap,
                                        float ykP, float ykI, float ykD, int yCap,
                                        float turnkP, float turnkI, float turnkD, int turnCap){
  xPID.kP = xkP; xPID.kI = xkI; xPID.kD = xkD; xPID.cap = xCap;
  yPID.kP = ykP; yPID.kI = ykI; yPID.kD = ykD; yPID.cap = yCap;
  turnPID.kP = turnkP; turnPID.kI = turnkI; turnPID.kD = turnkD; turnPID.cap = turnCap;                         
}

void autonomousControl::moveDrive(float x, float y, float turn){
  simp->frontLeft.spin(fwd, -(x*cos(simp->get_flbr()-tracking->getangleR()) + y*sin(simp->get_flbr()-tracking->getangleR())) - turn, voltageUnits::mV);
  simp->frontRight.spin(fwd, (x*cos(simp->get_frbl()-tracking->getangleR()) + y*sin(simp->get_frbl()-tracking->getangleR())) - turn, voltageUnits::mV);
  simp->backLeft.spin(fwd, -(x*cos(simp->get_frbl()-tracking->getangleR()) + y*sin(simp->get_frbl()-tracking->getangleR())) - turn, voltageUnits::mV);
  simp->backRight.spin(fwd, (x*cos(simp->get_flbr()-tracking->getangleR()) + y*sin(simp->get_flbr()-tracking->getangleR())) - turn, voltageUnits::mV);
}

float autonomousControl::averageRPM(){
  return (fabs(simp->frontRight.velocity(rpm)) + fabs(simp->frontLeft.velocity(rpm)) + fabs(simp->backRight.velocity(rpm)) + fabs(simp->backLeft.velocity(rpm)))/4;
}

float autonomousControl::updatePID(PIDSettings *good){
  good->error = good->curr - good->target;
  good->derivative = good->error - good->prevError;
  good->totalError = good->totalError + good->error;

  if((good->totalError*good->kI)>good->cap) good->totalError = good->cap/good->kI;
  else if((good->totalError*good->kI)<-good->cap)good->totalError = -good->cap/good->kI;

  if(std::signbit(good->error) != std::signbit(good->prevError)) good->totalError = 0;

  good->prevError = good->error;
  return -(good->kP*good->error + good->kD*good->derivative + good->kI*good->totalError);
}

int autonomousControl::turnCap(float distanceMag){
  if(distanceMag>30.0) return 2000;
  else if(distanceMag>10.0) return 6000;
  else return 10000;
}

void autonomousControl::movAB(){
  updateCurrPos();
  vectorD[0] = xPID.target - xPID.curr;
  vectorD[1] = yPID.target - yPID.curr;
  vMag = sqrt((vectorD[0]*vectorD[0]) + (vectorD[1]*vectorD[1])); 

  int turningCap = turnCap(vMag);

  float xVoltage = updatePID(&xPID);
  float yVoltage = updatePID(&yPID);
  float angleVoltage = updatePID(&turnPID);

  if(angleVoltage>turningCap) angleVoltage = turningCap;
  else if(angleVoltage<-turningCap) angleVoltage = -turningCap;

  if(xVoltage>10000) xVoltage = 10000;
  else if(xVoltage<-10000) xVoltage = -10000;

  if(yVoltage>10000) yVoltage = 10000;
  else if(yVoltage<-10000) yVoltage = -10000;

  moveDrive(xVoltage, yVoltage, angleVoltage);  
}

void autonomousControl::updateTargetPos(float x, float y, int angleO){
  xPID.target = x;
  yPID.target = y;
  turnPID.target = angleO;
}

void autonomousControl::updateIntakePct(int pow){ intakePct = pow; }

void autonomousControl::updateFlyTBH(){

  flyError = flyWheelRPM - simp->flyOuttake.velocity(rpm);
  flyVoltage += flykI*flyError;

  // Clip
  if (flyVoltage > 12000) flyVoltage = 12000;
  else if (flyVoltage < lowerBound) flyVoltage = lowerBound;

  // Zero crossing
  if (std::signbit(flyError) != std::signbit(flyLastError)){
    if( firstCross ){
      flyVoltage = flyApprox;
      firstCross = false;
    } else flyVoltage = 0.5 * (flyVoltage + flyZero);
    flyZero = flyVoltage;
  }
  flyLastError = flyError;
}

void autonomousControl::intakeMove(){
  simp->leftIntake.spin(fwd, intakePct, pct);
  simp->rightIntake.spin(fwd, -intakePct, pct);
}

void autonomousControl::flyMove(){
  updateFlyTBH();
  simp->flyOuttake.spin(fwd, flyVoltage, voltageUnits::mV);
}

void autonomousControl::rollerMove(){ 
  simp->rollerIntake.spin(fwd, rollerPct, pct); 
  }

void autonomousControl::waitUntilSettled(){
  wait(100, msec);
  while(averageRPM() != 0){
    wait(20, msec);
  }
}

void autonomousControl::waitTilFull(){
  wait(20, msec);
  int loopTime = simp->Brain.Timer.time(msec);

  while(simp->shootD.pressing() == false && (2000 > (simp->Brain.Timer.time() - loopTime))){
    wait(20, msec);
  }
}

void autonomousControl::waitUntilDistance(float dis){
  wait(100, msec);
  while(dis < vMag){
    wait(20, msec);
  }
}

void autonomousControl::waitUntilBalls(int ball){
}

void autonomousControl::waitUntilDeg(float deg){
  wait(100, msec);
  while(deg < fabs(turnPID.curr - turnPID.target) ){
    wait(20, msec);
  }
}

void autonomousControl::updateFly(int rpm){
  flyApprox = 10000;
  flyError = 0;
  flyLastError = 0;
  flyVoltage = 0;
  flyZero = 8000;
  lowerBound = 5000;
  firstCross = true;
  flyWheelRPM = rpm;
}

void autonomousControl::stopFly(){
  flyApprox = 0;
  flyError = 0;
  flyLastError = 0;
  flyVoltage = 0;
  flyZero = 0;
  lowerBound = 0;
  firstCross = true;
  flyWheelRPM = 0;
}

void autonomousControl::updateRoller(int pwr){ rollerPct = pwr; }

void autonomousControl::shootBall(int balls){
  ballsDeteced = 0;
  ballsToShoot = balls;
  shooting = true;
}

void autonomousControl::shootingBall(){
  if (shooting == true){
    rollerPct = 40;

    if ((prevShot == true) && (simp->shootD.pressing() == false)){
      ballsDeteced++;
      if (ballsDeteced == ballsToShoot){
        rollerPct = 0;
        shooting = false;
      }
    }
    prevShot = simp->shootD.pressing();
  }
}

void autonomousControl::autoMain(){
  simp->set_drive_break_type(coast);

  while(true){
    movAB();
    shootingBall();
    intakeMove();
    rollerMove();
    flyMove();
    task::sleep(20);
  }
}

void autonomousControl::updateCurrPos(){
  xPID.curr = tracking->getXPos();
  yPID.curr = tracking->getYPos();
  turnPID.curr = tracking->getangleD();
}
