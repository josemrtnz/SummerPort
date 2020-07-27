/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jose Martinez                                             */
/*    Created:      Sat Jul 04 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

int prevA = 0;
bool chungus = true;

void opControl(){
  //task debugPrint(updateScreen); //gg
  int *p;

  task autoStart(autoMain);

  updateTargetPos(-26, 52, 0);
  waitUntilDistance(20.0);
  updateIntakePct(100);
  waitUntilSettled();
  updateIntakePct(0);
  
  updateTargetPos(-26, 30, 0);
  waitUntilSettled();
  

  updateTargetPos(-6, 47.65, 30);
  task::sleep(50);
  waitUntilDistance(vMag/2);

  updateIntakePct(100);
  waitUntilSettled();
  updateIntakePct(0);
  
  updateTargetPos(-26, 30, 0);
  waitUntilSettled();

  updateTargetPos(-43, 48.5, -50);
  waitUntilDistance(10);
  betterPID();
  updateIntakePct(100);
  waitUntilBalls(2);
  updateIntakePct(0);

  /*
  updateTargetPos(-30, 21, -120);
  waitUntilSettled();
  updateTargetPos(-40, 11, -120);
  waitUntilSettled();

  updateTargetPos(10, 12, -90);
  waitUntilSettled();

  updateTargetPos(14, 14, -180);
  waitUntilSettled();

  updateTargetPos(14, 11, -180);
  waitUntilSettled();

  updateTargetPos(14, 20, -180);
  waitUntilSettled();

  updateTargetPos(25, 20, -90);
  waitUntilSettled();

  updateTargetPos(68, 20, -200);
  waitUntilSettled();

  updateTargetPos(71, 9, -200);
  waitUntilSettled();

  updateTargetPos(-20, 25, 0);
  waitUntilSettled();
  updateTargetPos(-5, 0, 0);
  waitUntilSettled();
  */
  autoStart.stop();

  while(1){

    if(chungus) driveXA();
    else driveX();

    intakeX();

    if(Controller1.ButtonL1.pressing()) chungus = true;
    else if(Controller1.ButtonL2.pressing()) chungus = false;

    if(Controller1.ButtonB.pressing()){ 
        updateTargetPos(1, 1, 0);
        waitUntilSettled();
      }
    if(Controller1.ButtonA.pressing()){
      //debugPrint.suspend();
      p = enterCoor();
      //debugPrint.resume();
      updateTargetPos(*p, *(p+1), 0);
      waitUntilSettled();
    }

    wait(30, msec);
  }
}

void autonM(){

}

void disabledR(){
  int max = 2;
  /*
  if(incSelect) autonSelect++;
  else if(decSelect) autonSelect--;
  */
  if(autonSelect == -1) autonSelect=max;
  if(autonSelect == max+1) autonSelect=0;

  if(autonSelect != prevA){
    switch(autonSelect){
    case 0:
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(color(255, 0, 0));
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("RED AUTO");
      break;

    case 1:
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(color(0, 255, 0));
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("GREEN AUTO");
      break;
    
    case 2:
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(color(0, 0, 255));
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("BLUE Auto");
      break;

    default:
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(white);
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("Select Auton");
    }
  }

  prevA = autonSelect;
  wait(100, msec);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Comp.drivercontrol(opControl);
  Comp.autonomous(autonM);

  while(1){
    if(!Comp.isEnabled()) disabledR();
    wait(100, msec);
  }
}
