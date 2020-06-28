#include "tracking.h"

using namespace std;


///////////////////////////////////////////
// Local variables
double xPos, yPos, currentAngle = 0;
double prevLeftEnc, prevRightEnc, prevBackEnc = 0;
double deltaL, deltaR, deltaS, deltaAngle = 0;
double prevAngle = 0;

double prevL, prevR = 0;
double currentL, currentR;

float angleD = 0;
double averageLR;
double pit = 3.14159265359; // (355/113) pi aproximation
double wheelCir = 4*pit;
///////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//This will be  run as a task so it will always be running in the background
//and its job is to keep track of where the robot is and it orientation.
void tracking(void* param){
  // This loop will update the x and y position of the bot
  std::uint32_t loopTime;

  while(true){

    //Current time at start of loop
    loopTime = millis();

    //Updates and stores current encoder values (In this case we are using the front motor encoders)
    currentL = topLeft.get_position();
    currentR = topRight.get_position();

    //Updates and stores current degree
    angleD = (gyroM.get_rotation()+90)/360;
    currentAngle = gyroM.get_rotation() - (360*angleD) + 90.0; //IMU starts off at 0 but the bot typically faces 90 degrees so we offset here by 90

    //Calculates difference between current and previous encoder values and then converts it into inches
    deltaL = wheelCir * (currentL - prevL)/360;
    deltaR = wheelCir * (currentR - prevR)/360;

    //Update and store previous encoder values
    prevL = currentL;
    prevR = currentR;

    //Calculates average delta values
    averageLR = (deltaL + deltaR)/2;

    currentAngle = 90; // <- Pay no attention to this. (was used for testing when I did not have acces to an imu)

    //Calculates x and y position
    xPos = xPos + (averageLR * cos((currentAngle*3.14159265)/180));
    yPos = yPos + (averageLR * sin((currentAngle*3.14159265)/180));

    //Delays task so it does not hog all resources
    Task::delay_until(&loopTime, 10);
  }
}
//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//This will be  run as a task so it will always be running in the background
//and its job is to keep track of where the robot is and it orientation.
void trackingPylons(void* param){
  std::uint32_t loopTime;

  // This loop will update the x and y position and angle the bot is facing
  while(true){

    //Current time at start of loop
    loopTime = millis();

    //The change in encoder values since last cycle in inches
    deltaL = wheelCir * (leftTracker.get_value() - prevLeftEnc)/360;
    deltaR = wheelCir * (rightTracker.get_value() - prevRightEnc)/360;
    deltaS = wheelCir * (backTracker.get_value() - prevBackEnc)/360;

    //Update previous value of the encoders
    prevLeftEnc = leftTracker.get_value();
    prevRightEnc = rightTracker.get_value();
    prevBackEnc = backTracker.get_value();

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
    float p = i + angleD;
    float cosP = cos(p);
    float sinP = sin(p);

    yPos += h*cosP;
    xPos += h*sinP;

    yPos += h2*(-sinP);
    xPos += h2*cosP;
    angleD += a;
    currentAngle = angleD * (180/pit);
    //Delays task so it does not hog all resources
    Task::delay_until(&loopTime, 10);
  }
}
//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//This will be  run as a task so it will always be running in the background
//and its job is to update the screen with xy coordinates and angle of the bot.
void updateScreen(void* param){

  //Clears the screen.
  master.clear();
  std::uint32_t loopTime;

  while(true){
    loopTime = millis();

    //Prints the x and y coordinates and angle the bot is facing to the lcd.
    lcd::print(1, "x: %lf y: %lf", xPos, yPos);
    lcd::print(2, "Angle: %lf", currentAngle);

    //Prints the x and y coordinates and angle the bot is facing to the Controller.
    master.print(0, 0, "x: %lf", xPos);
    Task::delay(50);
    master.print(1, 0, "y: %lf", yPos);
    Task::delay(50);
    master.print(2, 0, "Angle: %lf", currentAngle);

    //Delays task so it does not hog all resources
    Task::delay_until(&loopTime, 150);
  }
}
//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

                                      /*Getters*/
/***************************************************************************************************/
double getxPos(){
  return xPos;
}

double getyPos(){
  return yPos;
}

double getAngle(){
  return angleD;
}
/****************************************************************************************************/
