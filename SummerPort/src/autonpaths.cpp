#include "autonpaths.h"

chasisControl bigBot(0, 011);

void autonInit(float offset, float scalar){
  bigBot = chasisControl(offset, scalar);
}

int autoStart(){
  bigBot.coastMotor();

  while(true){
    bigBot.movAb();
    bigBot.intakeMovePct();
    task::sleep(20);
  }

  return 1;
}

void testPath(){

  task autoForReal(autoStart);

  bigBot.updateTargetPos(-26, 52, 0);
  bigBot.waitUntilDistance(20.0);
  bigBot.updateIntakePct(100);
  bigBot.waitUntilSettled();
  bigBot.updateIntakePct(0);
  
  bigBot.updateTargetPos(-26, 30, 0);
  bigBot.waitUntilSettled();
  

  bigBot.updateTargetPos(-5, 50, 35);
  task::sleep(50);
  bigBot.waitUntilDistance(bigBot.vMag/2);

  bigBot.updateIntakePct(100);
  bigBot.waitUntilSettled();
  bigBot.updateIntakePct(0);
  
  bigBot.updateTargetPos(-26, 30, 0);
  bigBot.waitUntilSettled();

  bigBot.updateTargetPos(-41, 48, -40);
  bigBot.waitUntilDistance(10);
  bigBot.betterPID();
  bigBot.updateIntakePct(100);
  //waitUntilBalls(2);
  task::sleep(2000);
  bigBot.updateIntakePct(0);

  
  bigBot.updateTargetPos(-30, 20, -150);
  bigBot.waitUntilDistance(25);
  bigBot.updateIntakePct(-100);
  task::sleep(750);
  bigBot.updateIntakePct(0);
  bigBot.waitUntilDistance(5);
  bigBot.updateIntakePct(100);
  bigBot.updateTargetPos(-42, 6, -150);
  task::sleep(2000);
  bigBot.updateIntakePct(0);


  bigBot.updateTargetPos(0, 30, -90);
  bigBot.waitUntilDistance(30);
  bigBot.updateIntakePct(-100);
  task::sleep(750);
  bigBot.updateIntakePct(0);

  bigBot.updateTargetPos(12, 22, -180);
  bigBot.waitUntilSettled();

  
  bigBot.updateTargetPos(12, 12, -180);
  bigBot.updateIntakePct(100);
  task::sleep(2000);
  bigBot.updateIntakePct(0);

  
  bigBot.updateTargetPos(12, 20, -180);
  bigBot.waitUntilDistance(2);

  
  bigBot.updateTargetPos(40, 30, -90);
  bigBot.waitUntilDistance(15);
  bigBot.updateIntakePct(-100);
  task::sleep(750);
  bigBot.updateIntakePct(0);

  bigBot.updateTargetPos(65, 30, -200);
  bigBot.waitUntilSettled();
  

  bigBot.updateTargetPos(67, 11, -200);
  bigBot.updateIntakePct(100);
  task::sleep(2000);
  bigBot.updateIntakePct(0);
  autoForReal.stop();
}