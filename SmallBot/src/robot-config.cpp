#include "vex.h"

using namespace vex;

// A global instance of brain and competition
brain  Brain;
competition Comp;

// VEXcode device constructors
controller Controller1 = controller(primary);

// Global instances of the drive motors
motor frontRight = motor(PORT8, ratio18_1, false);
motor frontLeft = motor(PORT7, ratio18_1, false);
motor backLeft = motor(PORT9, ratio18_1, false);
motor backRight = motor(PORT10, ratio18_1, false);

// Global instances of the intake motors
motor leftIntake = motor(PORT6, ratio18_1, true);
motor rightIntake = motor(PORT5, ratio18_1, false);

// Global instances of the tracking encoders
encoder leftTracker = encoder(Brain.ThreeWirePort.A);
encoder rightTracker = encoder(Brain.ThreeWirePort.G);
encoder backTracker = encoder(Brain.ThreeWirePort.C);

// Global instances of the auton selecors.
limit incSelect = limit(Brain.ThreeWirePort.E);
limit decSelect = limit(Brain.ThreeWirePort.F);

// Global instance of the ballDetector sensor
line ballDetector = line(Brain.ThreeWirePort.F);

// Global instance of the gyro sensor
inertial gyroM(PORT20);

float wheelDiameter = 2.85; //Tracking Wheel Diameter 2.785
double pi = 3.14159265359; // (355/113) pi aproximation
double wheelCir = wheelDiameter*pi; // Tracking Wheel Diameter
float sL = 2.5; // Distance to tracking center
float sR = 2.5; // Distance to tracking center
float sS = 8; // Distance to tracking center

float frblWheels = 2.35619449; // 135 deg in rads
float flbrWheels = 0.7853981634; // 45 deg in rads

// Tracking End Reult Variables
float xPos = 0;
float yPos = 0;
float angleD = 0;
float angleR = 0;

int autonSelect = 0;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // Tracking wheels are reset
  leftTracker.resetRotation();
  rightTracker.resetRotation();
  backTracker.resetRotation();

  wait(500, msec);
  // Gyro Callibrates
  gyroM.calibrate();
  while(gyroM.isCalibrating()){
    wait(50, msec);
  }

  // Prints the values of the tracking wheels in degrees.
  printf("%.0lf, %.0lf, %.0lf \n", leftTracker.position(deg), rightTracker.position(deg), backTracker.position(deg));

  // Tracking task is started here and will run non-stop.
  task trackingP(trackingPylons);
}