#pragma once

#include "api.h"
#include "odom.hpp"

namespace catlib {  

    enum class DriveType { 
        SPLIT_ARCADE,
        SINGLE_ARCADE,
        TANK
    };

    class Drivetrain {
        public:
            Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, double wheelDiameter, double driveRPM);
            pros::MotorGroup* leftMotors;
            pros::MotorGroup* rightMotors;
            double wheelDiameter;
            double driveRPM;
    };

    
}