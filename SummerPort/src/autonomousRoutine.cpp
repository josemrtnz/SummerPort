#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants(1500, 10, 5000, 4000, 
                             1500, 10, 5000, 4000,
                             800, 0, 3000, 0);
}

void autonomousRoutine::run() {
  control->updateTargetPos(0, 48, 0);
  control->waitUntilSettled();
  control->updateTargetPos(-20, 48, -90);
  control->waitUntilSettled();
  control->updateTargetPos(0, 48, 0);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  wait(8000, msec);
}