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
robotChasis simp = robotChasis(2.75, 5.5, 5.5, 5.2);
odometry tracker = odometry(&simp, 0, 0, 0);
autonomousControl autoChasis = autonomousControl(&simp, &tracker);
autonomousRoutine autoRoutine = autonomousRoutine(&autoChasis);

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
  startAuto.stop();
  userControl driveJose = userControl(&simp, true);
  driveJose.driveLoop();
}

void autonM(){
  simp.gyroM.resetRotation();
  autoRoutine.run(TEST);
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
