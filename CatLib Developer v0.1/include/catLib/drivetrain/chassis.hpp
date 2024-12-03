#include "main.h"

namespace catlib {
    class Chassis {
        public:
            Chassis(Drivetrain* drivetrain, PIDConstants* linearPID, PIDConstants* angularPID, OdomSensors* odomSensors, DriveType d);

            void calibrate();

            void setPose(double x, double y, double theta, bool isRadian = false);

            void setBrakeMode(pros::MotorBrake brakeMode);

            Vector2d getPose();

            Vector3d getPoseWithTheta(bool isRadian);

            void splitArcade();

            void singleArcade();

            void tankDrive();

            void setDrive(double l, double r);

            void driveStraightPID(double targetDistance, double speedCap = 1);

            void turnToHeadingPID(double targetDeg, double speedCap = 1);

            void driveToPoint(double x, double y, double timeOut, double maxVoltage, double minVoltage);

            void turnToPoint(double x, double y, double timeOut, double speedCap = 1);

            void follow(Curve path, double speedCap = 1, double timeOut, double lookAhead, bool reversed = false);

            Drivetrain* drivetrain;
            OdomSensors* odomSensors;
            PIDConstants* linearPID;
            PIDConstants* angularPID;
            DriveType d;
            Vector2d pose;
            double heading;
    };
}