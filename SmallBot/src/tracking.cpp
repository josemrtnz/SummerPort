#include "tracking.h"


///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// This will be  run as a task so it will always be running in the background
// and its job is to keep track of where the robot is and it orientation.
// xPos, yPos, angleR, and angleR are global variables and every file will have access to them.
int trackingPylons(){
  double loopTime;
  double deltaL;
  double deltaR;
  double deltaS;
  float prevLeftEnc = 0;
  float prevRightEnc = 0;
  float prevBackEnc = 0;
  

  // This loop will update the x and y position and angle the bot is facing
  while(true){

    //Current time at start of loop
    loopTime = Brain.Timer.time(msec);

    //The change in encoder values since last cycle in inches
    deltaL = wheelCir * (leftTracker.rotation(deg) - prevLeftEnc)/360;
    deltaR = wheelCir * (rightTracker.rotation(deg) - prevRightEnc)/360;
    deltaS = wheelCir * (backTracker.rotation(deg) - prevBackEnc)/360;

    //Update previous value of the encoders
    prevLeftEnc = leftTracker.rotation(deg);
    prevRightEnc = rightTracker.rotation(deg);
    prevBackEnc = backTracker.rotation(deg);

    float h;
    float i;
    float h2;

   
    float a = (deltaL - deltaR)/(sL + sR);

    if(a){
      float r = deltaR/a;
      i = a / 2.0;
      float sinI = sin(i);
      h = ((r + sR) * sinI) * 2.0;

      float r2 = deltaS/a;
      h2 = ((r2 + sS) * sinI) * 2.0;
    } else {
      h = deltaR;
      i = 0;
      h2 = deltaS;
    }
    float p = i + angleR;
    float cosP = cos(p);
    float sinP = sin(p);

    yPos += h*cosP;
    xPos += h*sinP;

    yPos += h2*(-sinP);
    xPos += h2*cosP;
    angleR += a;
    angleD = angleR * (180/pi);
    //Delays task so it does not hog all resources
    task::sleep(10 - (Brain.Timer.time(msec)-loopTime));
  }
  return 1;
}
//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// This will be  run as a task and its job is to display information about the bot 
// on the controller, v5 brain, and colsole
int updateScreen(){

  // Clears controller the screen.
  Controller1.Screen.clearScreen();
  double loopTime;

  while(true){
    loopTime = Brain.Timer.time(msec);

    // Prints the x and y coordinates and angle the bot is facing to the Controller.
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.print("x: %.1fin y: %.1fin     ", xPos, yPos);
    Controller1.Screen.newLine();
    Controller1.Screen.print("Angle: %.1f°    ", angleD);
    // Controller1.Screen.print("Drive mV: %.0lf");

    // Prints information about the bot to the console
    printf("Distance: %.2lf Y Voltage: %.0f X Voltage: %.0f\n", vMag, yVoltage, xVoltage);
    printf("Tracking Wheels Angle: %0.f   IMU angle: %0.lf\n", angleD, gyroM.rotation());
    printf("rightTW: %.0lf, leftTW: %.0lf, backTW: %.0lf\n\n\n", rightTracker.position(deg), leftTracker.position(deg), backTracker.position(deg));

    //Delays task so it does not hog all resources
    task::sleep(150 - (Brain.Timer.time(msec)-loopTime));
  }

  return 1;
}
//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
