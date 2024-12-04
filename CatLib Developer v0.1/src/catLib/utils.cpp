#include "main.h"

double catlib::toRadian(double degree) {
    return degree / 180 * M_PI;
}

double catlib::toDeg(double radian) {
    return radian / M_PI * 180;
}

double catlib::to0_360(double degree) {
    while (degree < 0 || degree > 360) {
        if (degree > 360) degree -= 360;
        else if (degree < 0) degree += 360;
    }
    return degree;
}

double catlib::toNegPos180(double degree) {
    while (degree < -180 || degree > 180) {
        if (degree > 180) degree -= 360;
        else if (degree < -180) degree += 360;
    }
    return degree;
}

double catlib::toNegPos90(double degree) {
    while (degree < -90 || degree > 90) {
        if (degree > 90) degree -= 180;
        else if (degree < -90) degree += 180;
    }
    return degree;
}

double catlib::left_velocity_scaling(double drive_output, double heading_output){
    double ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/12000.0;
    if (ratio > 1) {
        return (drive_output+heading_output)/ratio;
    }
    return drive_output+heading_output;
}

double catlib::right_velocity_scaling(double drive_output, double heading_output){
    double ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/12000.0;
    if (ratio > 1) {
        return (drive_output-heading_output)/ratio;
    }
    return drive_output-heading_output;
}

bool catlib::is_line_settled(float targetX, float targetY, float desired_angle_deg, float currentX, float currentY){
    return( (targetY-currentY) * cos(toRadian(desired_angle_deg)) <= -(targetX-currentX) * sin(toRadian(desired_angle_deg)));
}

double catlib::limit(double input, double min, double max) {
    if (input < min) return min;
    if (input > max) return max;
    return input;
}

double catlib::limit_min(double drive_output, double drive_min_voltage){
    if(drive_output < 0 && drive_output > -drive_min_voltage){
        return -drive_min_voltage;
    }
    if(drive_output > 0 && drive_output < drive_min_voltage){
        return drive_min_voltage;
    }
  return drive_output;
}