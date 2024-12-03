#include "main.h"

catlib::PIDConstants::PIDConstants(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

void catlib::PIDConstants::setPIDConstants(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

catlib::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, double wheelDiameter, double driveRPM) {
    this->leftMotors = leftMotors;
    this->rightMotors = rightMotors;
    this->wheelDiameter = wheelDiameter;
    this->driveRPM = driveRPM;
}