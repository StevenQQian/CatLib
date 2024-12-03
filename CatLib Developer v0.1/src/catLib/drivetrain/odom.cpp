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