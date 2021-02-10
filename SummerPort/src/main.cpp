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
robotChasis simp = robotChasis(3.2, 2.1, 2.1, 7.4);
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
  
  autoChasis.setPIDConstants(2000, 10, 6000, 5000, 
                             2000, 10, 6000, 5000,
                             600, 0, 1800, 0);
  autoChasis.updateTargetPos(10, 10, 90);
  autoChasis.waitUntilSettled();
  autoChasis.updateTargetPos(0, 0, 0);
  wait(8000, msec);
  
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
