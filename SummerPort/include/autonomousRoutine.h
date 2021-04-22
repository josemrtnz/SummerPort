#pragma once

#include "chasisControl.h"

class autonomousRoutine {
  public:
    void run(int autoSelection);
    autonomousRoutine(autonomousControl *autoControl);

  private:
    autonomousControl *control;
    void odometryOnlyAuto();
    void test();
    void odometryVisionAuto();
};