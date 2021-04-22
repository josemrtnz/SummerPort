#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants( 750, 25, 2000, 4000, 
                            750, 25, 2000, 4000,
                            200, 20, 1200, 2000);
}

void autonomousRoutine::run(int autoSelection) {
  switch(autoSelection){
    case 0:
      test();
      break;
    case 1:
      odometryOnlyAuto();
      break;
    case 2:
      odometryVisionAuto();
      break;
    default:
      printf("No Auto Selected!");
      break;
  }
}

void autonomousRoutine::test(){
  control->updateTargetPos(0, 0, 180);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
}

void autonomousRoutine::odometryOnlyAuto(){
  // First Goal
  control->updateIntakePct(-100);
  wait(1000, msec);
  control->updateTargetPos(0, 9, 0);
  control->updateFly(600);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(1);
  control->updateTargetPos(0, 9, 90);
  control->waitUntilDistance(2);
  control->updateIntakePct(0);
  control->updateTargetPos(16, 10, 90);
  control->waitUntilDistance(2);
  control->updateTargetPos(18, 8, 130);
  control->waitUntilDeg(5);
  control->waitUntilDistance(2);
  control->shootBall(1);
  wait(1000, msec);

  // Second Goal
  control->updateTargetPos(11, 7, 130);
  control->waitUntilDistance(2);
  control->updateTargetPos(11, 7, 0);
  control->waitUntilDeg(2);
  control->updateTargetPos(11, 61, 0);
  control->waitUntilDistance(30);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(10, 61, 90);
  control->waitUntilDeg(1);
  control->updateTargetPos(14, 61, 90);
  control->waitUntilDistance(1);
  control->updateIntakePct(0);
  control->shootBall(1);
  wait(1000, msec);

  // Third Goal
  control->updateTargetPos(9, 62, 0);
  control->waitUntilDeg(5);
  control->updateTargetPos(5, 98, 0);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->updateTargetPos(5, 103, 90);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDeg(5);
  control->updateTargetPos(16, 103, 90);
  control->waitUntilDistance(2);
  control->updateTargetPos(10, 99, 45);
  control->waitUntilDeg(2);
  control->updateIntakePct(0);
  control->updateTargetPos(12, 116, 45);
  control->waitUntilDistance(1);
  control->shootBall(1);
  wait(1250, msec);

  // Fourth Goal
  control->updateTargetPos(15, 106, -90);
  control->waitUntilDeg(5);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->updateTargetPos(-10, 106, -90);
  control->waitUntilDistance(2);
  control->updateTargetPos(-33, 102, -90);
  control->waitUntilDistance(1);
  control->updateTargetPos(-32, 107.5, 2);
  control->updateIntakePct(-10);
  control->waitUntilDeg(2);
  control->shootBall(1);
  wait(1250, msec);

  // Fifth Goal
  control->updateTargetPos(-33, 104, -90);
  control->waitUntilDeg(2);
  control->updateTargetPos(-69, 104, -90);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(1);
  control->updateTargetPos(-84, 110, -45);
  control->updateIntakePct(0);
  control->waitUntilDistance(2);
  control->shootBall(1);
  wait(1250, msec);

  // Sixth Goal
  control->updateTargetPos(-78, 100, -90);
  control->waitUntilDistance(2);
  control->updateTargetPos(-82, 100, -180);
  control->waitUntilDeg(5);
  control->updateTargetPos(-82, 62, -180);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(-82, 54, -90);
  control->waitUntilDeg(2);
  control->updateTargetPos(-84, 54, -90);
  control->updateIntakePct(0);
  control->waitUntilDistance(2);
  control->shootBall(1);
  wait(1250, msec);

  // 7th Goal
  control->updateTargetPos(-82, 62, -180);
  control->waitUntilDeg(2);
  control->updateTargetPos(-82, 27, -180);
  control->updateRoller(30);
  control->updateIntakePct(-100);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(-82, 21, -90);
  control->waitUntilDeg(5);
  control->updateTargetPos(-88, 21, -90);
  control->waitUntilDistance(2);
  control->updateIntakePct(-10);
  control->updateTargetPos(-82, 11, -135);
  control->waitUntilDeg(5);
  control->shootBall(1);
  wait(1250, msec);

  // 8th Goal
  control->updateTargetPos(-90, 11, -270);
  control->waitUntilDeg(2);
  control->updateRoller(30);
  control->updateIntakePct(-100);
  control->waitTilFull();
  control->updateRoller(0);
  control->updateTargetPos(-42, 14, -270);
  control->waitUntilDistance(2);
  control->updateTargetPos(-42, 14, -180);
  control->updateIntakePct(0);
  control->waitUntilDeg(2);
  control->updateTargetPos(-42, 11, -180);
  control->waitUntilDistance(2);
  control->shootBall(1);
  wait(1250, msec);
}

void autonomousRoutine::odometryVisionAuto(){
  
}