#pragma once

#include "api.h"

namespace catlib {
    namespace omniWheel {
        constexpr double OMNI_275 = 2.75;
        constexpr double OMNI_2 = 2.125;
        constexpr double OMNI_325 = 3.25;
        constexpr double OMNI_4 = 4;
    }

    class TrackingWheel{
        public:
            TrackingWheel(pros::Rotation* rotation, double wheelDiameter, double offset);

            void reset();

            double distanceTraveled();

        private:
            pros::Rotation* rotation = nullptr;
            double wheelDiameter;
            double offset;
    };

    class OdomSensors {
        public:
            OdomSensors(pros::Imu* inertial, TrackingWheel* vertical = nullptr, TrackingWheel* horizontal = nullptr);
            TrackingWheel* vertical = nullptr;
            TrackingWheel* horizontal = nullptr;
            pros::Imu* inertial = nullptr;
    };

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