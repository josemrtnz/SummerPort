#include "tracking.h"


odometry::odometry(robotChasis *robot, float x, float y, float deg){
  simp = robot;
  xPos = x;
  yPos = y;
  angleD = deg;
  angleR = 0;
}

float odometry::getangleD(){ return angleD; }
float odometry::getangleR(){ return angleR; }
float odometry::getXPos(){ return xPos; }
float odometry::getYPos(){ return yPos; }

int odometry::updatePosition(){
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
    loopTime = simp->Brain.Timer.time(msec);

    //The change in encoder values since last cycle in inches
    deltaL = simp->getWheelCir() * (simp->leftTracker.rotation(deg) - prevLeftEnc)/360;
    deltaR = simp->getWheelCir() * (simp->rightTracker.rotation(deg) - prevRightEnc)/360;
    deltaS = simp->getWheelCir() * (simp->backTracker.rotation(deg) - prevBackEnc)/360;

    //Update previous value of the encoders
    prevLeftEnc = simp->leftTracker.rotation(deg);
    prevRightEnc = simp->rightTracker.rotation(deg);
    prevBackEnc = simp->backTracker.rotation(deg);

    float h;
    float i;
    float h2;

   
    float a = (deltaL - deltaR)/(simp->getsL() + simp->getsR());

    if(a){
      float r = deltaR/a;
      i = a / 2.0;
      float sinI = sin(i);
      h = ((r + simp->getsR()) * sinI) * 2.0;

      float r2 = deltaS/a;
      h2 = ((r2 + simp->getsS()) * sinI) * 2.0;
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
    angleD = angleR * (180/simp->getPI());
    //Delays task so it does not hog all resources
    task::sleep(5 - (simp->Brain.Timer.time(msec)-loopTime));
  }
  return 1;
}

int odometry::updateScreen(){
    // Clears controller the screen.
  simp->Controller1.Screen.clearScreen();
  double loopTime;

  while(true){
    loopTime = simp->Brain.Timer.time(msec);

    // Prints the x and y coordinates and angle the bot is facing to the Controller.
    simp->Controller1.Screen.setCursor(0, 0);
    simp->Controller1.Screen.print("x: %.1fin y: %.1fin     ", xPos, yPos);
    simp->Controller1.Screen.newLine();
    simp->Controller1.Screen.print("Angle: %.1f°    ", angleD);
    // Controller1.Screen.print("Drive mV: %.0lf");

    // Prints information about the bot to the console
    //printf("Distance: %.2lf Y Voltage: %.0f X Voltage: %.0f\n", vMag, yVoltage, xVoltage);
    printf("Tracking Wheels Angle: %0.f   IMU angle: %0.lf\n", angleD, simp->gyroM.rotation());
    printf("rightTW: %.0lf, leftTW: %.0lf, backTW: %.0lf\n", simp->rightTracker.position(deg), simp->leftTracker.position(deg), simp->backTracker.position(deg));
    printf("Flywheel RPM: %.1lf, Flywheel Voltage: %.0lf\n\n\n", simp->flyOuttake.velocity(rpm), simp->flyOuttake.voltage(voltageUnits::mV));
    //printf("%.0lf, %.0lf, %.0lf \n", Brain.Timer.time(msec), flyOuttake.velocity(rpm), flyOuttake.voltage(voltageUnits::mV));

    //Delays task so it does not hog all resources
    task::sleep(200 - (simp->Brain.Timer.time(msec)-loopTime));
  }

  return 1;
}