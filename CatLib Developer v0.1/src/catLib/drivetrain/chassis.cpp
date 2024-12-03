#include "main.h"

catlib::Chassis::Chassis(Drivetrain* drivetrain, PIDConstants* linearPID, PIDConstants* angularPID, OdomSensors* odomSensors, DriveType d) {
    this->drivetrain = drivetrain;
    this->linearPID = linearPID;
    this->angularPID = angularPID;
    this->odomSensors = odomSensors;
    this->d = d;
}

void catlib::Chassis::setBrakeMode(pros::MotorBrake brakeMode) {
    this->drivetrain->leftMotors->set_brake_mode_all(brakeMode);
    this->drivetrain->rightMotors->set_brake_mode_all(brakeMode);
}

void catlib::Chassis::calibrate() {
    this->odomSensors->inertial->tare_rotation();
    this->odomSensors->horizontal->reset();
    this->odomSensors->vertical->reset();
}

Vector2d catlib::Chassis::getPose() {
    return this->pose;
}

Vector3d catlib::Chassis::getPoseWithTheta(bool isRadian = false) {
    double currHeading = this->heading;
    if (isRadian) {
        currHeading = currHeading / 180 * M_PI;
    }
    Vector3d currPose(this->pose[0], this->pose[1], currHeading);
    return currPose;
}