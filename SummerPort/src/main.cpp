/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jose Martinez                                             */
/*    Created:      Sat Jul 04 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//your mom 
#include "vex.h"

using namespace vex;

int prevA = 0;
bool chungus = true;

void opControl(){
  task debugPrint(updateScreen); //gg
  int *p;

  movAb(-22.5, 50, 0, 3500);
  movAb(-22.5, 35, 0, 1500);
  movAb(-3, 50, 30, 2000);

  movAb(-43, 51, -30, 4000);

  movAb(-30, 21, -120, 2000);
  movAb(-40, 11, -120, 2000);

  movAb(10, 12, -90, 1000);

  movAb(14, 14, -180, 2000);

  movAb(14, 11, -180, 1000);

  movAb(14, 20, -180, 1500);

  movAb(25, 20, -90, 1200);

  movAb(68, 20, -200, 4000);

  movAb(71, 9, -200, 2000);

  while(1){

    if(chungus) driveXA();
    else driveX();

    if(Controller1.ButtonL1.pressing()) chungus = true;
    else if(Controller1.ButtonL2.pressing()) chungus = false;

    if(Controller1.ButtonB.pressing()){ 
        movAb(-20, 25, 0, 3500);
        movAb(-5, 0, 0, 2000);
      }
    if(Controller1.ButtonA.pressing()){
      debugPrint.suspend();
      p = enterCoor();
      debugPrint.resume();
      movAb(*p, *(p+1), 0, 2000);
    }

    wait(30, msec);
  }
}

void autonM(){

}

void disabledR(){
  int max = 2;
  if(incSelect) autonSelect++;
  else if(decSelect) autonSelect--;

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
