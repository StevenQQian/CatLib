#include "main.h"

namespace catlib {
    class Chassis {
        public:
            Chassis(Drivetrain* drivetrain, PIDConstants* linearPIDConstants, PIDConstants* angularPIDConstants, OdomSensors* odomSensors, DriveType d);

            void calibrate();

            int track();

            void setPose(double x, double y, double theta = -10000000, bool isRadian = false);

            void setBrakeMode(pros::MotorBrake brakeMode);

            Vector2d getPose();

            Vector3d getPoseWithTheta(bool isRadian);

            void arcadeDrive(double linear, double angular);

            void tankDrive(double l, double r);

            void setDrive(double l, double r);

            void driveStraightPID(double targetDistance, double speedCap = 1, double timeOut = 5000);

            void turnToHeadingPID(double targetDeg, double speedCap = 1);

            void driveToPoint(double x, double y, double timeOut, double maxVoltage, double minVoltage);

            void turnToPoint(double x, double y, double timeOut, double speedCap = 1);

            void follow(Curve path, double timeOut, double lookAhead, double speedCap = 1, bool reversed = false);

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