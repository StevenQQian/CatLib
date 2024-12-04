#include "main.h"

catlib::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, double wheelDiameter, double driveRPM) {
    this->leftMotors = leftMotors;
    this->rightMotors = rightMotors;
    this->wheelDiameter = wheelDiameter;
    this->driveRPM = driveRPM;
}