#include "tracking.h"


odometry::odometry(robotChasis *robot, double x, double y, double deg){
  simp = robot;
  xPos = x;
  yPos = y;
  angleD = deg;
  angleR = 0;
}

double odometry::getangleD(){ return angleD; }
double odometry::getangleR(){ return angleR; }
double odometry::getXPos(){ return xPos; }
double odometry::getYPos(){ return yPos; }

int odometry::updatePosition(){
  double loopTime;
  double deltaL;
  double deltaR;
  double deltaS;
  double prevLeftEnc = 0;
  double prevRightEnc = 0;
  double prevBackEnc = 0;
  double currLeftEnc = 0;
  double currRightEnc = 0;
  double currBackEnc = 0;
  

  // This loop will update the x and y position and angle the bot is facing
  while(true){

    //Current time at start of loop
    loopTime = simp->Brain.Timer.time(msec);

    currLeftEnc = simp->leftTracker.position(deg);
    currRightEnc = simp->rightTracker.position(deg);
    currBackEnc = simp->backTracker.position(deg);

    //The change in encoder values since last cycle in inches
    deltaL = (currLeftEnc - prevLeftEnc) * simp->getWheelCir()/360;
    deltaR = (currRightEnc - prevRightEnc)* simp->getWheelCir()/360;
    deltaS = (currBackEnc - prevBackEnc)* simp->getWheelCir()/360;

    //Update previous value of the encoders
    prevLeftEnc = currLeftEnc;
    prevRightEnc = currRightEnc;
    prevBackEnc = currBackEnc;

    double h;
    double i;
    double h2;

   
    double a = (deltaL - deltaR)/(simp->getsL() + simp->getsR());
    //double a = (angleD - simp->gyroM.rotation(deg)) * (simp->getPI()/180);

    if(a){
      double r = deltaR/a;
      i = a / 2.0;
      double sinI = sin(i);
      h = (r + simp->getsR()) * sinI * 2.0;

      double r2 = deltaS/a;
      h2 = (r2 + simp->getsS()) * sinI * 2.0;
    } else {
      h = deltaR;
      i = 0;
      h2 = deltaS;
    }
    double p = i + angleR;
    double cosP = cos(p);
    double sinP = sin(p);

    yPos += h*cosP;
    xPos += h*sinP;

    yPos += h2*(-sinP);
    xPos += h2*cosP;
    angleR += a;
    angleD = angleR * (180/simp->getPI());
    //Delays task so it does not hog all resources
    task::sleep(8 - (simp->Brain.Timer.time(msec)-loopTime));
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
    //simp->Controller1.Screen.print("right: %.1lf     ", simp->rightTracker.rotation(deg));
    simp->Controller1.Screen.newLine();
    simp->Controller1.Screen.print("Angle: %.1f°    ", angleD);
    //simp->Controller1.Screen.print("left: %.1lf     ", simp->leftTracker.rotation(deg));
    simp->Controller1.Screen.newLine();
    //simp->Controller1.Screen.print("back: %.1lf     ", simp->backTracker.rotation(deg));
    // Controller1.Screen.print("Drive mV: %.0lf");

    // Prints information about the bot to the console
    //printf("Distance: %.2lf Y Voltage: %.0f X Voltage: %.0f\n", vMag, yVoltage, xVoltage);
    printf("Tracking Wheels Angle: %0.f   IMU angle: %0.lf\n", angleD, simp->gyroM.rotation(deg));
    printf("rightTW: %.0lf, leftTW: %0.lf, backTW: %.0lf\n", simp->rightTracker.position(deg), simp->leftTracker.position(deg), simp->backTracker.position(deg));
    printf("Flywheel RPM: %.1lf, Flywheel Voltage: %.0lf\n\n\n", simp->flyOuttake.velocity(rpm), simp->flyOuttake.voltage(voltageUnits::mV));
    //printf("%.0lf, %.0lf, %.0lf \n", Brain.Timer.time(msec), flyOuttake.velocity(rpm), flyOuttake.voltage(voltageUnits::mV));

    //Delays task so it does not hog all resources
    task::sleep(200 - (simp->Brain.Timer.time(msec)-loopTime));
  }

  return 1;
}