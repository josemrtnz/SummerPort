#pragma once
#include "v5.h"
#include "v5_vcs.h"
using namespace vex;

/// Class for robot configuration.
///
/// A class for storing motor, sensor, brain, controller objects.
/// This class also holds most constants used throughout the program.
/// Classes \ref odometry, autonomousControl, and userControl will use
/// this class to access sensor data and motors.
class robotChasis{
  public:
    /// Gets pi value.
    ///
    /// PI value is derived from the pi approximation 355/113.
    /// @return pi value as a double
    double getPI();

    /// Gets left tracking wheel position.
    ///
    /// Current left tracking wheel position in degree units will be aqquired using this method.
    /// @return value of sL as a float.
    float getsL();

    /// Gets right tracking wheel position.
    ///
    /// Current right tracking wheel position in degree units will be aqquired using this method.
    /// @return value of sR as a float.
    float getsR();

    /// Gets back tracking wheel position.
    ///
    /// Current back tracking wheel position in degree units will be aqquired using this method.
    /// @return value of sS as a float.
    float getsS();

    /// Gets frbl value.
    ///
    /// Front right and back left wheel constant of 135 degrees.
    /// @return value frbl as a float
    float get_frbl();

    /// Gets flbr value.
    ///
    /// Front left and back right wheel constant of 45 degrees.
    /// @return value flbr as a float
    float get_flbr();

    /// Gets wheel circumference value of the tracking wheels.
    ///
    /// Wheel circumference of the right, left, back tracking wheels.
    /// @return value wheelCir as a double
    double getWheelCir();
    
    /// V5 Brain Instance 
    brain Brain;

    /// Competition Instance
    competition Comp;

    /// Controller Instance
    controller Controller1;

    /// Front Right Motor Instance
    motor frontRight = motor(PORT9, ratio18_1, true);

    /// Front Left Motor Instance
    motor frontLeft = motor(PORT8, ratio18_1, true);

    /// Back Left Motor Instance
    motor backLeft = motor(PORT1, ratio18_1, true);

    /// Back Right Motor Instance
    motor backRight = motor(PORT11, ratio18_1, true);

    /// Left Intake Motor Instance
    motor leftIntake = motor(PORT15, ratio36_1, true);

    /// Right Intake Motor Instance
    motor rightIntake = motor(PORT21, ratio36_1, true);

    /// Fly Wheel Motor Instance
    motor flyOuttake = motor(PORT14, ratio36_1, true);

    /// Roller Motor Instance
    motor rollerIntake = motor(PORT10, ratio36_1, true);

    //encoder leftTracker = encoder(Brain.ThreeWirePort.E);
    //encoder rightTracker = encoder(Brain.ThreeWirePort.A);
    //encoder backTracker = encoder(Brain.ThreeWirePort.C);
    /// Left Tracker Rotation Sensor Instance
    rotation leftTracker = rotation(PORT19, true);

    /// Right Tracker Rotation Sensor Instance
    rotation rightTracker = rotation(PORT20, false);

    /// Back Tracker Rotation Sensor Instance
    rotation backTracker = rotation(PORT18, false);
    
    /// Limit Switch Sensor Instance
    limit shootD = limit(Brain.ThreeWirePort.H);

    /// IMU Sensor Instance
    inertial gyroM = inertial(PORT12);

    /// Sets the brake type for the drive motors.
    ///
    /// Will set each drive motor to the brake type provided in the parameter field.
    /// @param B the type of break you want your to set your motors to.
    /// \see <a href="https://api.vexcode.cloud/v5/html/namespacevex.html#a2090f7d57d63c5cf693f269bc73568f1">vex::brakeType</a>
    void set_drive_break_type(brakeType B);

    /// Stops all drive motors.
    ///
    /// Will brake all drive motors according their current brakeType.
    /// \see <a href="https://api.vexcode.cloud/v5/html/namespacevex.html#a2090f7d57d63c5cf693f269bc73568f1">vex::brakeType</a>
    void stopMotors();

    /// Constructor for the class \ref robotChasis.
    ///
    /// This constructor will create an instance of the class \ref robotChasis.
    /// It will set values to wheelDiameter, sL, sR, and sS.
    /// @param wD Wheel diameter in inches.
    /// @param tcL Left Tracking Wheel distance to tracking center. 
    /// @param tcR Right Tracking Wheel distance to tracking center. 
    /// @param tcB Back Tracking Wheel distance to tracking center. 
    robotChasis(float wD, float tcL, float tcR, float tcB);

  private:

    float wheelDiameter;
    double wheelCir;
    const double PI = 22/7;

    float sL;
    float sR;
    float sS;

    float frblWheels = 2.35619449;
    float flbrWheels = 0.7853981634;

    int autonSelect;
};

void  vexcodeInit( robotChasis *simp );