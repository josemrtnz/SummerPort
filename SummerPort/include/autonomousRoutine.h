#pragma once

#include "chasisControl.h"

class autonomousRoutine {
  public:
    void run();
    void test();
    autonomousRoutine(autonomousControl *autoControl);

  private:
    autonomousControl *control;

};