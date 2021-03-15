#pragma once

#include "chasisControl.h"

class autonomousRoutine {
  public:
    void run();
    autonomousRoutine(autonomousControl *autoControl);

  private:
    autonomousControl *control;

};