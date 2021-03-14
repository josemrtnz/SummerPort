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
robotChasis simp = robotChasis(3.0, 5.5, 5.5, 5);
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
  
  autoChasis.setPIDConstants(1500, 10, 5000, 4000, 
                             1500, 10, 5000, 4000,
                             800, 0, 3000, 0);
                             
  autoChasis.updateTargetPos(0, 48, 0);
  autoChasis.waitUntilSettled();
  autoChasis.updateTargetPos(-20, 48, -90);
  autoChasis.waitUntilSettled();
  autoChasis.updateTargetPos(0, 48, 0);
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
