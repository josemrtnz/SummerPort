#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants( 1500, 25, 4500, 4000, 
                            1000, 25, 4500, 4000,
                            200, 20, 1200, 2000);
}

void autonomousRoutine::run() {
  control->updateTargetPos(0, 3, 0);
  control->updateFly(600);
  control->updateIntakePct(-100);
  control->updateRoller(40);
  control->waitTilFull();
  control->updateRoller(0);
  control->updateTargetPos(0, 3, 90);
  control->waitUntilDistance(2);
  control->updateIntakePct(0);
  control->updateTargetPos(15.5, 3, 120);
  control->waitUntilDistance(1);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(12, 3, 130);
  control->waitUntilDistance(2);
  control->updateTargetPos(12, 3, 0);
  control->waitUntilDeg(5);
  control->updateTargetPos(12, 62, 0);
  control->waitUntilDistance(30);
  control->updateIntakePct(-100);
  control->updateRoller(40);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(15, 62, 90);
  control->waitUntilDeg(2);
  control->updateIntakePct(0);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(14, 62, 0);
  control->waitUntilDeg(5);
  control->updateTargetPos(12, 98, 0);
  control->updateIntakePct(-100);
  control->updateRoller(40);
  control->updateTargetPos(12, 98, 90);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDeg(5);
  control->updateTargetPos(21, 98, 90);
  control->waitUntilDistance(2);
  control->updateIntakePct(0);
  control->updateTargetPos(21, 98, 48);
  control->waitUntilDeg(2);
  control->updateTargetPos(22, 111, 48);
  control->waitUntilDistance(1);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(22, 108, -90);
  control->waitUntilDeg(5);
  control->updateIntakePct(-100);
  control->updateRoller(40);
  control->waitTilFull();
  control->updateRoller(0);
  control->updateTargetPos(-10, 108, -90);
  control->waitUntilDistance(2);
  control->updateTargetPos(-33, 108, -90);
  control->waitUntilDistance(2);
  control->updateTargetPos(-33, 110, 2);
  control->updateIntakePct(-10);
  control->waitUntilDeg(2);
  control->shootBall(1);
  wait(1000, msec);

  control->updateTargetPos(-33, 109, -90);
  control->waitUntilDeg(2);
  control->updateTargetPos(-69, 109, -90);
  control->updateIntakePct(-100);
  control->updateRoller(40);
  control->waitTilFull();
  control->updateRoller(0);
  control->waitUntilDistance(2);
  control->updateTargetPos(-87.5, 114, -35);
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
  control->updateRoller(40);
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
  control->updateRoller(40);
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
  control->updateRoller(40);
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
}