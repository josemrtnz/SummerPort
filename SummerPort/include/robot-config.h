using namespace vex;

extern brain Brain;
extern competition Comp;

// VEXcode devices
extern controller Controller1;

extern motor frontRight;
extern motor frontLeft;
extern motor backLeft;
extern motor backRight;

extern motor leftIntake;
extern motor rightIntake;

extern encoder leftTracker;
extern encoder rightTracker;
extern encoder backTracker;

//extern limit incSelect;
//extern limit decSelect;

extern line ballDetector;

extern inertial gyroM;

extern float wheelRadius; //JK it's diameter
extern double wheelCir;
extern double pi;

extern float sL;
extern float sR;
extern float sS;

extern float frblWheels;
extern float flbrWheels;

extern float xPos;
extern float yPos;
extern float angleD;
extern float angleR;

extern int autonSelect;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );