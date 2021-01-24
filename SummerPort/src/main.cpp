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
autonomousControl autoChasis = autonomousControl(&simp, &tracker);

int trackerWrapper(){
  tracker.updatePosition();
  return 0;
}

int printerWrapper(){
  tracker.updateScreen();
  return 0;
}

int autoWrapper(){
  autoChasis.autoMain();
  return 0;
}

task startTracking(trackerWrapper);
task startPrinting(printerWrapper);
task startAuto(autoWrapper);

void opControl(){
  /*
  autoChasis.setPIDConstants(700, 1, 350, 1500, 
                             700, 1, 350, 1500, 
                             175, 40, 85, 500);
  autoChasis.updateTargetPos(15, 15, 90);
  autoChasis.waitUntilSettled();
  autoChasis.updateTargetPos(0, 0, 0);
  wait(100000, msec);
  */
  startAuto.stop();


  userControl driveJose = userControl(&simp, true);
  driveJose.driveLoop();
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
