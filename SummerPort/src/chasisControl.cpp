#include "chasisControl.h"

autonomousControl::autonomousControl(robotChasis *robot, odometry *tr){
  simp = robot;
  tracking = tr;
}

void autonomousControl::setPIDConstants(float flkP, float flkI, float flkD, int flCap,
                                        float frkP, float frkI, float frkD, int frCap,
                                        float blkP, float blkI, float blkD, int blCap,
                                        float brkP, float brkI, float brkD, int brCap,
                                        float rpmkP, float rpmkI, float rpmkD, int rpmCap,
                                        float turnkP, float turnkI, float turnkD, int turnCap){
  flPID.kP = flkP; flPID.kI = flkI; flPID.kD = flkD; flPID.cap = flCap;
  frPID.kP = frkP; frPID.kI = frkI; frPID.kD = frkD; frPID.cap = frCap;
  blPID.kP = blkP; blPID.kI = blkI; blPID.kD = blkD; blPID.cap = blCap;
  brPID.kP = brkP; brPID.kI = brkI; brPID.kD = brkD; brPID.cap = brCap;
  rpmPID.kP = rpmkP; rpmPID.kI = rpmkI; rpmPID.kD = rpmkD; rpmPID.cap = rpmCap;
  turnPID.kP = turnkP; turnPID.kI = turnkI; turnPID.kD = turnkD; turnPID.cap = turnCap;                         
}

void autonomousControl::moveDrive(float fr, float fl, float br, float bl){
  simp->frontLeft.spin( fwd, fl, voltageUnits::mV);
  simp->frontRight.spin(fwd, fr, voltageUnits::mV);
  simp->backLeft.spin(fwd, bl, voltageUnits::mV);
  simp->backRight.spin(fwd, br, voltageUnits::mV);
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
  else if(distanceMag<10.0) return 6000;
  else return (-200*distanceMag) + 8000;
}

void autonomousControl::movAB(){
  vectorD[0] = targetPos[0] - currPos[0];
  vectorD[1] = targetPos[1] - currPos[1];

  vMag = sqrt((vectorD[0]*vectorD[0]) + (vectorD[1]*vectorD[1])); 
  updateCurrPos();
  updateDriveRPM(vectorD[0]/vMag, vectorD[1]/vMag);
  driveVoltages[0] = updatePID(&frPID);
  driveVoltages[1] = updatePID(&flPID);
  driveVoltages[2] = updatePID(&brPID);
  driveVoltages[3] = updatePID(&blPID);

  moveDrive(driveVoltages[0], driveVoltages[1], driveVoltages[2], driveVoltages[3]);  
}

void autonomousControl::updateTargetPos(float x, float y, int angleO){
  targetPos[0] = x;
  targetPos[1] = y;
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
  simp->rightIntake.spin(fwd, intakePct, pct);
}

void autonomousControl::flyMove(){
  updateFlyTBH();
  simp->flyOuttake.spin(fwd, flyVoltage, voltageUnits::mV);
}

void autonomousControl::rollerMove(){ simp->rollerIntake.spin(fwd, rollerPct, pct); }

void autonomousControl::waitUntilSettled(){
  wait(100, msec);
  while(averageRPM() != 0){
    wait(20, msec);
  }
}

void autonomousControl::waitTilFull(){
  wait(100, msec);
  while(simp->shootD.pressing() == false){
    wait(20, msec);
  }
  wait(500, msec);
}

void autonomousControl::waitUntilDistance(float dis){
  wait(50, msec);
  while(dis < vMag){
    wait(20, msec);
  }
}

void autonomousControl::waitUntilBalls(int ball){
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
    rollerPct = 100;

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

void autonomousControl::driveRatio(float x, float y){
  driveRatioV[0] = x*cos(simp->get_frbl()-currPos[2]) + y*sin(simp->get_frbl()-currPos[2]);
  driveRatioV[1] = -(x*cos(simp->get_flbr()-currPos[2]) + y*sin(simp->get_flbr()-currPos[2]));
  driveRatioV[2] = x*cos(simp->get_frbl()-currPos[2]) + y*sin(simp->get_frbl()-currPos[2]);
  driveRatioV[3] = -(x*cos(simp->get_flbr()-currPos[2]) + y*sin(simp->get_flbr()-currPos[2]));
}

void autonomousControl::updateDriveRPM( float x, float y ){
  driveRatio(x, y);
  rpmPID.target = vMag;
  turnPID.curr = currPos[3];
  float genRPM = updatePID(&rpmPID);
  float turnRPM = updatePID(&turnPID);

  if (fabs(turnRPM) > 120) turnRPM = (turnRPM/fabs(turnRPM))*120;
  if(genRPM > 282) genRPM = 282;

  driveRPM[0] = (driveRatioV[0] * genRPM) - turnRPM;
  driveRPM[1] = (driveRatioV[1] * genRPM) - turnRPM;
  driveRPM[2] = (driveRatioV[2] * genRPM) - turnRPM;
  driveRPM[3] = (driveRatioV[3] * genRPM) - turnRPM;
  
  float max = findMaxRPM();

  if (max > 200) normalizeRPM(max);
  set_RPM_values();
}

void autonomousControl::set_RPM_values(){
  frPID.target = driveRPM[0];
  flPID.target = driveRPM[1];
  brPID.target = driveRPM[2];
  blPID.target = driveRPM[3]; 

  frPID.target = simp->frontRight.velocity(rpm);
  flPID.target =simp->frontLeft.velocity(rpm);
  brPID.target = simp->backRight.velocity(rpm);
  blPID.target = simp->backLeft.velocity(rpm); 
}

void autonomousControl::updateCurrPos(){
  currPos[0] = tracking->getXPos();
  currPos[1] = tracking->getYPos();
  currPos[2] = tracking->getangleR();
  currPos[3] = tracking->getangleD();
}

float autonomousControl::findMaxRPM(){
  float max = 0;
  for (int i = 0; i < 4; i++){ 
        if (driveRPM[i] > max) max = driveRPM[i];
        } 
  return max;
}

void autonomousControl::normalizeRPM(float max){
  float divisor = max/200;
  driveRPM[0] = driveRPM[0]/divisor;
  driveRPM[1] = driveRPM[1]/divisor;
  driveRPM[2] = driveRPM[2]/divisor;
  driveRPM[3] = driveRPM[3]/divisor;
}