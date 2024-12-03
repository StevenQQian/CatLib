#include "main.h"

catlib::TrackingWheel::TrackingWheel(pros::Rotation* rotation, double wheelDiameter, double offset) {
    this->rotation = rotation;
    this->wheelDiameter = wheelDiameter;
    this->offset = offset;
}

void catlib::TrackingWheel::reset() {
    this->rotation->reset_position();
}

double catlib::TrackingWheel::distanceTraveled() {
    return this->rotation->get_position() / 36000 * wheelDiameter * M_PI;
}

catlib::OdomSensors::OdomSensors(pros::Imu* inertial, TrackingWheel* vertical = nullptr, TrackingWheel* horizontal = nullptr) {
    this->inertial = inertial;
    this->vertical = vertical;
    this->horizontal = horizontal;
}

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