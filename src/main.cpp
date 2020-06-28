#include "main.h"

// Runs initialization code. This occurs as soon as the program is started.

void initialize() {

	//Initializes the v5 brain screen
	lcd::initialize();

	//Starts the tracking task that will run every 10ms
	Task cTracking (trackingPylons, (void*)"PROS", TASK_PRIORITY_DEFAULT,
                2*TASK_STACK_DEPTH_DEFAULT, "Update_position");


  //Starts the updateScreen task that will run every 20ms
	Task screenUpdate (updateScreen, (void*)"PROS", TASK_PRIORITY_DEFAULT,
							  TASK_STACK_DEPTH_DEFAULT, "Update_screen");
}

// Runs while the robot is in the disabled state of Field Management System or the VEX Competition Switch.
void disabled() {}

// Runs after initialize(), and before autonomous when connected to the Field
// Management System or the VEX Competition Switch. (Autonmous selector is goes here)
void competition_initialize() {}

//Runs the user autonomous code after competition_initialize.
void autonomous() {
	movAb(0.0, -20.0);
	Task::delay(3000);
}

// Runs the operator control code following the initialize().
void opcontrol() {
	while (true) {
		pros::delay(20);
	}
}
