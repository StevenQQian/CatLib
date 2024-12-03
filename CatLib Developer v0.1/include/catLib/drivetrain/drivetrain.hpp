#pragma once

#include "api.h"

namespace catlib {
    struct PIDf {
        PIDf(double kP, double kI, double kD, double kF = 0, bool signFlipReset = true, double antiWindup = 0);
        void setConstants(double kP, double kI, double kD, double kF = 0);
        
        double kP, kI, kD, kF;
        bool signFlipReset;
        double antiWindup;
    };

    enum class DriveType { 
        SPLIT_ARCADE,
        SINGLE_ARCADE,
        TANK
    };

    class Drivetrain {
        public:
            Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, double wheelDiameter, double driveRPM, OdomSensors* odomSensors, DriveType d);

            void calibrate();

            void setPose(double x, double y, double theta, bool isRadian = false);

            void driveStraightPID(double targetDistance);
        private:
            pros::MotorGroup* leftMotors;
            pros::MotorGroup* rightMotors;
            OdomSensors* odomSensors;
            double wheelDiameter;
            double driveRPM;
    };
}