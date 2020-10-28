/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jose Martinez                                             */
/*    Created:      Sat Jul 04 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

int prevA = 0; // Used for Autonous Selection

void opControl(){ // Driver Controll Function
  task debugPrint(updateScreen); // Starts a task that will print information about the bot to the controller and console.
  int *p; // A pointer, used for later.

  task autoStart(autoMain); // Starts a task that will controll the bot during the autonomous period.
   
  /* This is a temporary autonomous routine
  updateTargetPos(5, 0, 0);
  waitUntilSettled();
  updateTargetPos(5, 5, 0);
  waitUntilSettled();
  updateTargetPos(0, 5, 0);
  waitUntilSettled();
  updateTargetPos(0, 0, 0);
  waitUntilSettled();
  
  updateTargetPos(-26, 52, 0);
  waitUntilDistance(20.0);
  updateIntakePct(100);
  waitUntilSettled();
  updateIntakePct(0);
  
  updateTargetPos(-26, 30, 0);
  waitUntilSettled();
  

  updateTargetPos(-5, 50, 35);
  waitUntilDistance(vMag/2);

  updateIntakePct(100);
  waitUntilSettled();
  updateIntakePct(0);
  
  updateTargetPos(-26, 30, 0);
  waitUntilSettled();

  updateTargetPos(-41, 48, -40);
  waitUntilDistance(10);
  betterPID();
  updateIntakePct(100);
  //waitUntilBalls(2);
  task::sleep(2000);
  updateIntakePct(0);

  
  updateTargetPos(-30, 20, -150);
  waitUntilDistance(25);
  updateIntakePct(-100);
  task::sleep(750);
  updateIntakePct(0);
  waitUntilDistance(5);
  updateIntakePct(100);
  updateTargetPos(-42, 6, -150);
  task::sleep(2000);
  updateIntakePct(0);


  updateTargetPos(0, 30, -90);
  waitUntilDistance(30);
  updateIntakePct(-100);
  task::sleep(750);
  updateIntakePct(0);

  updateTargetPos(12, 22, -180);
  waitUntilSettled();

  
  updateTargetPos(12, 12, -180);
  updateIntakePct(100);
  task::sleep(2000);
  updateIntakePct(0);

  
  updateTargetPos(12, 20, -180);
  waitUntilDistance(2);

  
  updateTargetPos(40, 30, -90);
  waitUntilDistance(15);
  updateIntakePct(-100);
  task::sleep(750);
  updateIntakePct(0);

  updateTargetPos(65, 30, -200);
  waitUntilSettled();
  

  updateTargetPos(67, 11, -200);
  updateIntakePct(100);
  task::sleep(2000);
  updateIntakePct(0);

  

  updateTargetPos(-20, 25, 0);
  waitUntilSettled();
  updateTargetPos(-5, 0, 0);
  waitUntilSettled();
*/
  autoStart.stop(); // Ends the autonomous task

  bool driveMASelect = false; // driveMASelect defaults to false meaning the drive will start in relative mode.

  while(1){ // Starts the drive loop. 
  
    if(driveMASelect) driveMA(); // If driveMASelect is true the driveMA will run
    else driveM(); // Otherwise driveM will run

    intakeM(); // Conrtolls the intake

    if(Controller1.ButtonL1.pressing()) driveMASelect = true; // Pressing L1 on the controller will make driveMASelect true.
    else if(Controller1.ButtonL2.pressing()) driveMASelect = false; // Pressing L2 on the controller will make driveMASelect false.

    /* Ignore this
    if(Controller1.ButtonB.pressing()){
        updateTargetPos(1, 1, 0);
        waitUntilSettled();
      }
    if(Controller1.ButtonA.pressing()){
      //debugPrint.suspend();
      p = enterCoor();
      //debugPrint.resume();
      updateTargetPos(*p, *(p+1), 0);
      waitUntilSettled();
    }*/
    wait(30, msec);
  }
}

void autonM(){ // Final autonomous code will be here

}

void disabledR(){ // This the code that will be run when the bot is in the disabled mode
  int max = 3; // Number of autonous routines we can select
  
  if(incSelect) autonSelect++; // If you press the limit switch labeled for incSelect, the autonSelect variable will be increased by one.
  else if(decSelect) autonSelect--; // If you press the limit switch labeled for decSelect, the autonSelect variable will be decreased by one.
  
  // This will make it so that if your autonSelect reaches -1, it will instead set it to last autonroutine, essenctially making it loop back.
  if(autonSelect == -1) autonSelect = max-1; 
  // This will make it so that if your autonSelect reaches your max, it will instead set it to first autonroutine, essenctially making it loop to the start.
  if(autonSelect == max) autonSelect = 0;

  // If your current autonSelect does not equal to your previous autonSelect, then you will run the following code, which will update the brian screen.
  if(autonSelect != prevA){
    switch(autonSelect){ // Depending on what value your autonSelect is, it will run one of the cases below.

    case 0: // If your auton select is 0, Auton 0 will run.

      // This block of code will set the screen to a red color and place some text saying "RED AUTO"
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(color(255, 0, 0));
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("RED AUTO");

      break; // Terminates this case and exits the switch statement

    case 1: // If your auton select is 1, Auton 1 will run.

      // This block of code will set the screen to a green color and place some text saying "GREEN AUTO"
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(color(0, 255, 0));
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("GREEN AUTO");

      break; // Terminates this case and exits the switch statement.
    
    case 2: // If your auton select is 2, Auton 2 will run.

      // This block of code will set the screen to a blue color and place some text saying "BLUE AUTO"
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(color(0, 0, 255));
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("BLUE Auto");

      break; // Terminates this case and exits the switch statement.

    default: // If somehow the auton select is not one of the values above, this case will run.

      // This block of code will set the screen to a white color and place some text saying "Select Auto"
      Brain.Screen.clearScreen();
      Brain.Screen.setFillColor(white);
      Brain.Screen.drawRectangle(0, 0, 480, 272);
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(2, 4);
      Brain.Screen.print("Select Auton");


      break; // Terminates this case and exits the switch statement.
    }
  }

  prevA = autonSelect; // Sets our prevA to autonSelect
  wait(100, msec);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Comp.drivercontrol(opControl); // opControl will run when driverControl is enabled.
  Comp.autonomous(autonM); // autonM will run when autonomous is enabled

  while(1){
    if(!Comp.isEnabled()) disabledR(); // disabledR will keep running as long as the bot is disabled.
    wait(100, msec);
  }
}
