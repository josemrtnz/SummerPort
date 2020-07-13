#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
competition Comp;

// VEXcode device constructors
controller Controller1 = controller(primary);

motor frontRight = motor(PORT4, ratio18_1, false);
motor frontLeft = motor(PORT1, ratio18_1, false);
motor backLeft = motor(PORT2, ratio18_1, false);
motor backRight = motor(PORT3, ratio18_1, false);

encoder leftTracker = encoder(Brain.ThreeWirePort.A);
encoder rightTracker = encoder(Brain.ThreeWirePort.G);
encoder backTracker = encoder(Brain.ThreeWirePort.C);

limit incSelect = limit(Brain.ThreeWirePort.E);
limit decSelect = limit(Brain.ThreeWirePort.F);

inertial gyroM(PORT20);

float wheelRadius = 3; //JK its diameter 2.785
double pi = 3.14159265359; // (355/113) pi aproximation
double wheelCir = wheelRadius*pi;
float sL = 6.75;
float sR = 6.75;
float sS = 9;

float frblWheels = 2.35619449;
float flbrWheels = 0.7853981634;

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
  // nothing to initialize
  leftTracker.resetRotation();
  rightTracker.resetRotation();
  backTracker.resetRotation();

  wait(500, msec);
  gyroM.calibrate();
  while(gyroM.isCalibrating()){
    wait(50, msec);
  }

  printf("%.0lf, %.0lf, %.0lf \n", leftTracker.position(deg), rightTracker.position(deg), backTracker.position(deg));
  task trackingP(trackingPylons);

  
}