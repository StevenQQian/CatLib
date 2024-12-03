#pragma once

#include "main.h"

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
            OdomSensors(TrackingWheel* vertical, TrackingWheel* horizontal, pros::Imu* inertial);

        private:
            TrackingWheel* vertical = nullptr;
            TrackingWheel* horizontal = nullptr;
            pros::Imu* inertial = nullptr;
    };

    class PIDConstants {
        public:
            PIDConstants(double kP, double kI, double kD);
        private:
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