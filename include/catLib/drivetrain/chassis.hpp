#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "odom.hpp"
#include "../pursuit/curve.hpp"
#include "pid.hpp"

namespace catlib {
    class Chassis {
        public:
            Chassis(Drivetrain* drivetrain, PIDConstants* linearPIDConstants, PIDConstants* angularPIDConstants, OdomSensors* odomSensors, DriveType d);
            
            Chassis();

            void calibrate();

            void track();

            void setPose(double x, double y, double theta, bool isRadian);

            void setBrakeMode(pros::MotorBrake brakeMode);

            Vector2d getPose();

            Vector3d getPoseWithTheta(bool isRadian);

            void arcadeDrive(double linear, double angular);

            void tankDrive(double l, double r);

            void setDrive(double l, double r);

            void driveStraightPID(double targetDistance, double speedCap, double timeOut);

            void turnToHeadingPID(double targetDeg, double speedRatio, bool reversed);

            void driveToPoint(double x, double y, double timeOut, double maxVoltage, double minVoltage);

            void turnToPoint(double x, double y, double timeOut, double speedCap);

            void driveWithAngle(double targetDistance, double targetDeg, double timeOut, double speedCap);

            void follow(Curve path, double timeOut, double lookAhead, double speedRatio, bool reversed);

            Drivetrain* drivetrain;
            OdomSensors* odomSensors;
            PIDConstants* linearPIDConstants;
            PIDConstants* angularPIDConstants;
            PID linearPID;
            PID angularPID;
            DriveType d;
            Vector2d pose;
            double heading;
    };
}