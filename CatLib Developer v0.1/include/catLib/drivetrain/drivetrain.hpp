#pragma once

#include "api.h"
#include "odom.hpp"

namespace catlib {

    class PIDConstants {
        public:
            PIDConstants(double kP, double kI, double kD);

            void setPIDConstants(double kP, double kI, double kD);
            double kP;
            double kI;
            double kD;
    };

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