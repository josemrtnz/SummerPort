#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants( 1500, 25, 4500, 4000, 
                            1000, 25, 4500, 4000,
                            200, 20, 1200, 2000);
}

void autonomousRoutine::run() {
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
  control->updateTargetPos(17, 8, 130);
  control->waitUntilDeg(5);
  control->waitUntilDistance(1);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(10, 7, 130);
  control->waitUntilDistance(2);
  control->updateTargetPos(10, 7, 0);
  control->waitUntilDeg(2);
  control->updateTargetPos(11, 62, 0);
  control->waitUntilDistance(30);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(12.5, 62, 90);
  control->waitUntilDeg(2);
  control->updateIntakePct(0);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(12, 62, 0);
  control->waitUntilDeg(5);
  control->updateTargetPos(12, 98, 0);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->updateTargetPos(10, 100, 90);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDeg(5);
  control->updateTargetPos(18, 100, 90);
  control->waitUntilDistance(2);
  control->updateTargetPos(12, 99, 45);
  control->waitUntilDeg(2);
  control->updateIntakePct(0);
  control->updateTargetPos(14, 116, 42);
  control->waitUntilDistance(2);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(20, 106, -90);
  control->waitUntilDeg(5);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->updateTargetPos(-10, 106, -90);
  control->waitUntilDistance(2);
  control->updateTargetPos(-33, 102, -90);
  control->waitUntilDistance(2);
  control->updateTargetPos(-37, 106.5, 2);
  control->updateIntakePct(-10);
  control->waitUntilDeg(2);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(-33, 104, -90);
  control->waitUntilDeg(2);
  control->updateTargetPos(-69, 104, -90);
  control->updateIntakePct(-100);
  control->updateRoller(30);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(-87.5, 108, -35);
  control->updateIntakePct(0);
  control->waitUntilDistance(2);
  control->shootBall(1);
  wait(1000, msec);

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
  control->updateTargetPos(-82, 62, -90);
  control->waitUntilDeg(2);
  control->updateTargetPos(-87, 62, -90);
  control->updateIntakePct(0);
  control->waitUntilDistance(2);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(-82, 62, -180);
  control->waitUntilDeg(2);
  control->updateTargetPos(-82, 27, -180);
  control->updateRoller(30);
  control->updateIntakePct(-100);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(-82, 27, -90);
  control->waitUntilDeg(5);
  control->updateTargetPos(-93, 27, -90);
  control->waitUntilDistance(2);
  control->updateIntakePct(-10);
  control->updateTargetPos(-90, 11, -135);
  control->waitUntilDeg(5);
  control->shootBall(1);
  wait(1000, msec);

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
}

void autonomousRoutine::test(){
  control->updateTargetPos(0, 0, 90);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 90);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 90);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 90);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 90);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
  
}