#pragma once

#include "chasisControl.h"

/// Class for creating autonomous routines.
///
/// A class for creating and selecting autonomous routines.
/// This class uses an /ref autonomousControl pointer to make autonmous routines.
class autonomousRoutine {
  public:

    /// Runs the selected autonomous routine.
    ///
    /// This function will run an autonomous routine depending on what the value of autoSelection is.
    /// @param autoSelection Number that represents which autonomous routine to run.
    void run(int autoSelection);

    /// Constructor for the class \ref autonomousRoutine.
    ///
    /// This constructor will create an instance of the class \ref autonomousRoutine.
    /// It will save the pointer to an \ref automousControl object for creation of autonmous routines.
    /// @param *autoControl Pointer of an \ref autonomousControl object.
    autonomousRoutine(autonomousControl *autoControl);

  private:
    autonomousControl *control;
    void odometryOnlyAuto();
    void test();
    void odometryVisionAuto();
};