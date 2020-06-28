#include "global.h"

Motor topRight(3, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor topLeft(10, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor botLeft(14, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor botRight(11, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);

Motor leftI(12, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor rightI(2, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

Motor lift(20, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);

float chasisAngle = gyroM.get_rotation();

Imu gyroM(5);
ADIAnalogIn liftPot('H');

Controller master(E_CONTROLLER_MASTER);

ADIEncoder leftTracker('A', 'B', false);
ADIEncoder rightTracker('C', 'D', true);
ADIEncoder backTracker('E', 'F', false);

float wheelRadius = 2;
float sL = 8.22;
float sR = 8.22;
float sS = 7.44;
