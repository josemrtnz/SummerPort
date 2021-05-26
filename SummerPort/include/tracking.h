#pragma once
#include "robot-config.h"
#include <math.h>
#include <stdio.h>

/// Class for robot odometry and debugging.
///
/// A class that will estimate pose using three rotation/encoder sensors.
/// This class holds information about the X and Y position in inches relative to the starting position.
/// This class also holds current orientation of the robot in degrees and radians.
/// This class uses the \ref robotChasis class to access sensor data and VEX V5 Periphials.
/// Classes \ref autonomousControl and userControl will use
/// this class to access the robots position.
class odometry{
public:

  /// Updates xPos, yPos, angleR, and angleD.
  ///
  /// This function will continually run updating the X position, Y position, and orientation of the robot.
  /// The algorithm used to estimate the pose and orientation can be found below.
  /// @return 1
  /// \see <a href="http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf">Pylons Tracking</a>
  int updatePosition();

  /// Prints information about the robot.
  ///
  /// Prints information to console and VEX V5 periphials.
  /// @return 1
  int updateScreen();

  /// Gets xPos value.
  ///
  /// Will return x position of the robot in inches. 
  /// @return xPos in inches
  double getXPos();

  /// Gets yPos value.
  ///
  /// Will return y position of the robot in inches. 
  /// @return yPos in inches
  double getYPos();

  /// Gets angleR value.
  ///
  /// Will return orientation of the robot in radians. 
  /// @return angleR in radians
  double getangleR();

  /// Gets angleD value.
  ///
  /// Will return orientation of the robot in degrees. 
  /// @return angleD in degrees
  double getangleD();

  /// Constructor for the class \ref sodometry.
  ///
  /// This constructor will create an instance of the class \ref odometry.
  /// It will set values to the starting positions xPos, yPos, and angleD.
  /// @param *robot Pointer to an instance of \ref robotChasis object.
  /// @param x X starting position in inches.
  /// @param y Y starting position in inches. 
  /// @param deg Degrees starting position. 
  odometry(robotChasis *robot, double x, double y, double deg);

private:
  double xPos;
  double yPos;
  double angleR;
  double angleD;
  robotChasis *simp;
};