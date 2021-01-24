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
robotChasis simp = robotChasis(3.3, 2.1, 2.1, 7.3);
odometry tracker = odometry(&simp, 0, 0, 0);

int trackerWrapper(){
  tracker.updatePosition();
  return 0;
}

int printerWrapper(){
  tracker.updateScreen();
  return 0;
}

void opControl(){
  task startTracking(trackerWrapper);
  task startPrinting(printerWrapper);
}

void autonM(){
}

void disabledR(){
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit(&simp);
  simp.Comp.drivercontrol(opControl);
  simp.Comp.autonomous(autonM);

  while(1){
    if(!simp.Comp.isEnabled()) disabledR();
    wait(100, msec);
  }
}
