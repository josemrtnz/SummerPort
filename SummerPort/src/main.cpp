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

void opControl(){
  task debugPrint(updateScreen); //gg
  int *p;
/*
  movAb(-22.5, 50, 0, 2000);
  movAb(-22.5, 35, 0, 2000);
  movAb(-3, 50, 30, 2000);
  movAb(-43, 51, -30, 2000);
  movAb(-30, 21, -120, 2000);
  movAb(-40, 11, -120, 2000);
  movAb(-12, 12, -90, 2000);
  movAb(14, 13, -180, 2000);
  movAb(14, 7, -180, 2000);
  movAb(68, 7, -225, 2000);
*/

  while(1){
    driveX();

    if(Controller1.ButtonB.pressing()) movAb(0, 0, 0, 2000);
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

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Comp.drivercontrol(opControl);
  Comp.autonomous(autonM);

  while(1){

    wait(100, msec);
  }
}
