#include "catLib/drivetrain/chassis.hpp"
#include "catLib/utils.hpp"
#include "catLib/drivetrain/pid.hpp"

catlib::Chassis::Chassis(Drivetrain* drivetrain, PIDConstants* linearPIDConstants, PIDConstants* angularPIDConstants, OdomSensors* odomSensors, DriveType d) {
    this->drivetrain = drivetrain;
    this->linearPIDConstants = linearPIDConstants;
    this->angularPIDConstants = angularPIDConstants;
    this->odomSensors = odomSensors;
    PID linearPID = PID(linearPIDConstants, PIDType::LINEAR);
    PID angularPID = PID(angularPIDConstants, PIDType::ANGULAR);

    this->angularPID = angularPID;
    this->linearPID = linearPID;

    this->d = d;
}

void catlib::Chassis::setBrakeMode(pros::MotorBrake brakeMode) {
    this->drivetrain->leftMotors->set_brake_mode_all(brakeMode);
    this->drivetrain->rightMotors->set_brake_mode_all(brakeMode);
}

Vector2d catlib::Chassis::getPose() {
    return this->pose;
}

Vector3d catlib::Chassis::getPoseWithTheta(bool isRadian = false) {
    double currHeading = this->heading;
    if (isRadian) {
        currHeading = currHeading / 180 * M_PI;
    }
    Vector3d currPose(this->pose[0], this->pose[1], currHeading);
    return currPose;
}

void catlib::Chassis::track() {
    double currentVertical = this->odomSensors->vertical->distanceTraveled();
    double currentHorizontal = this->odomSensors->horizontal->distanceTraveled();
    double prevVertical = currentVertical;
    double prevHorizontal = currentHorizontal;
    double currentHeading = this->odomSensors->inertial->get_rotation();
    double prevHeading = currentHeading;
    while (1) {
        currentHeading = this->odomSensors->inertial->get_rotation();
        double deltaHeading = currentHeading - prevHeading;
        double avgHeading = (currentHeading + prevHeading) / 2;
        double currentVertical = this->odomSensors->vertical->distanceTraveled();
        double currentHorizontal = this->odomSensors->horizontal->distanceTraveled();
        double deltaY = currentVertical - prevVertical;
        double deltaX = currentHorizontal - prevHorizontal;
        double localX;
        double localY;
        if (deltaHeading == 0) {
            localX = deltaX;
            localY = deltaY;
        }
        else {
            localX = 2 * sin(catlib::toRadian(deltaHeading) / 2) * ((deltaX / (catlib::toRadian(deltaHeading))) + this->odomSensors->horizontal->offset);
            localY = 2 * sin(catlib::toRadian(deltaHeading) / 2) * ((deltaY / (catlib::toRadian(deltaHeading))) + this->odomSensors->vertical->offset);
        }
        double localPolarAngle;
        double localPolarLength;

        if (localX == 0 && localY == 0) {
            localPolarAngle = 0;
            localPolarLength = 0;
        }
        else {
            localPolarAngle = atan2(localY, localX);
            localPolarLength = sqrt(pow(localX, 2) + pow(localY, 2));
        }
        double globalPolarAngle = localPolarAngle - toRadian(prevHeading) - (toRadian(deltaHeading) / 2);
        this->pose[0] += localPolarLength * cos(globalPolarAngle);
        this->pose[1] += localPolarLength * sin(globalPolarAngle);
        this->heading = currentHeading;
        prevHeading = currentHeading;
        prevHorizontal = currentHorizontal;
        prevVertical = currentVertical;
        pros::delay(5);
    }
}

void catlib::Chassis::calibrate() {
    this->odomSensors->horizontal->reset();
    this->odomSensors->vertical->reset();
    this->odomSensors->inertial->reset();
    this->odomSensors->inertial->tare();
    this->pose[0] = 0;
    this->pose[1] = 0;
}

void catlib::Chassis::setPose(double x, double y, double theta = -10000000, bool isRadian = false) {
    if (theta != -10000000) {
        if (isRadian) {
            this->odomSensors->inertial->set_rotation(theta);
        }
    }
    this->pose[0] = x;
    this->pose[1] = y;
}

void catlib::Chassis::setDrive(double l, double r) {
    this->drivetrain->leftMotors->move_voltage(l);
    this->drivetrain->rightMotors->move_voltage(r);
}

void catlib::Chassis::driveStraightPID(double targetDistance, double speedCap = 1, double timeOut = 5000) {
    this->linearPID.reset();
    this->angularPID.reset();
    double error = targetDistance;
    double targetDeg = this->odomSensors->inertial->get_rotation();
    double time = 0;
    double distance = this->odomSensors->vertical->distanceTraveled();
    double prevDistance = distance;
    double distanceTraveled = 0;
    while ((fabs(error) > 0.2 || (this->drivetrain->leftMotors->get_actual_velocity() * this->drivetrain->wheelDiameter / 6000 * M_PI) > 0.06) && time <= timeOut) {
        distance = this->odomSensors->vertical->distanceTraveled();
        double deltaDistance = distance - prevDistance;
        distanceTraveled += deltaDistance;
        error = targetDistance - distanceTraveled;
        double driveOutput = this->linearPID.output(error);
        double degError = targetDeg - this->odomSensors->inertial->get_rotation();
        double turnOutput = this->angularPID.output(degError);
        driveOutput = catlib::limit(driveOutput, -12000 * speedCap, 12000 * speedCap);
        turnOutput = catlib::limit(turnOutput, -12000 * speedCap, 12000 * speedCap);
        this->setDrive(left_velocity_scaling(driveOutput, turnOutput), right_velocity_scaling(driveOutput, turnOutput));
        prevDistance = distance;
        time += 10;
        pros::delay(10);
    }
    this->setDrive(0, 0);
}

void catlib::Chassis::turnToHeadingPID(double heading, double speedRatio, bool reversed = 0) {
    this->angularPID.reset();
    double error = heading - this->odomSensors->inertial->get_rotation();
    double prevError = error;
    double deltaError = error - prevError;

    while (fabs(error) > 0.2 || fabs(deltaError) > 0.05) {
        error = heading - this->odomSensors->inertial->get_rotation() - reversed * 180;
        double driveOutput = this->angularPID.output(error);
        if (fabs(driveOutput) > 12000 && driveOutput > 0) driveOutput = 12000;
        else if (fabs(driveOutput) > 12000 && driveOutput < 0) driveOutput = -12000;
        driveOutput *= speedRatio;
        setDrive(driveOutput, -driveOutput);
        pros::delay(10);
    }
    setDrive(0, 0);
}

void catlib::Chassis::driveToPoint(double x, double y, double timeOut, double maxVoltage, double minVoltage) {
    this->linearPID.reset();
    this->angularPID.reset();
    Vector2d targetPose(x, y);
    double time = 0;
    double targetDeg = toDeg(atan2(x - this->pose[0], y - this->pose[1]));
    double angularError = targetDeg - to0_360(this->odomSensors->inertial->get_rotation());
    double driveError = (targetPose - this->pose).norm();
    bool isLineSettled = is_line_settled(x, y, targetDeg, this->pose[0], this->pose[1]);

    while (!isLineSettled && time <= timeOut) {
        driveError = (targetPose - this->pose).norm();
        isLineSettled = is_line_settled(x, y, targetDeg, this->pose[0], this->pose[1]);
        angularError = toNegPos180(toDeg(atan2(x - this->pose[0], y = this->pose[1])) - to0_360(this->odomSensors->inertial->get_rotation()));
        double driveOutput = this->linearPID.output(driveError);
        double headingScaleFactor = cos(toRadian(angularError));
        driveOutput *= headingScaleFactor;
        if (fabs(driveError) < 5) {
            angularError = 0;
        }
        angularError = toNegPos90(angularError);
        double turnOutput = this->angularPID.output(angularError);
        driveOutput = limit(driveOutput, -fabs(headingScaleFactor) * maxVoltage, fabs(headingScaleFactor) * maxVoltage);
        turnOutput = limit(turnOutput, maxVoltage, minVoltage);
        driveOutput = limit_min(driveOutput, minVoltage);
        this->setDrive(left_velocity_scaling(driveOutput, turnOutput), right_velocity_scaling(driveOutput, turnOutput));
        time += 10;
        pros::delay(10);
    }
    this->setDrive(0, 0);
}

void catlib::Chassis::turnToPoint(double x, double y, double timeOut, double speedCap = 1) {
    this->linearPID.reset();
    this->angularPID.reset();
    Vector2d targetPose = Vector2d(x, y);
    double targetDeg = toNegPos180(toDeg(atan2((targetPose - this->getPose())[0], (targetPose - this->getPose())[1])));
    double angularError = toNegPos180(targetDeg - to0_360(this->odomSensors->inertial->get_rotation()));
    double prevAngularError = angularError;
    double deltaError = angularError - prevAngularError;
    while (fabs(angularError) > 0.3 || fabs(deltaError) > 0.3) {
        targetDeg = toNegPos180(toDeg(atan2((targetPose - this->getPose())[0], (targetPose - this->getPose())[1])));
        angularError = toNegPos180(targetDeg - to0_360(this->odomSensors->inertial->get_rotation()));
        double driveOutput = this->angularPID.output(angularError);
        driveOutput = limit(driveOutput, -12000 * speedCap, 12000 * speedCap);
        this->setDrive(driveOutput, -driveOutput);
        pros::delay(10);
    }
    this->setDrive(0, 0);
}

void catlib::Chassis::driveWithAngle(double targetDistance, double targetDeg, double timeOut, double speedCap) {
    this->linearPID.reset();
    this->angularPID.reset();
    double angularError = toNegPos180(targetDeg - to0_360(this->odomSensors->inertial->get_rotation()));
    double prevAngularError = angularError;
    double deltaError = angularError - prevAngularError;
    double driveError = targetDistance;
    double distance = this->odomSensors->vertical->distanceTraveled();
    double prevDistance = distance;
    double distanceTraveled = 0;
    double time = 0;
    while ((fabs(driveError) > 0.2 || (this->drivetrain->leftMotors->get_actual_velocity() * this->drivetrain->wheelDiameter / 6000 * M_PI)) && time <= timeOut) {
        distance = this->odomSensors->vertical->distanceTraveled();
        double deltaDistance = distance - prevDistance;
        distanceTraveled += deltaDistance;
        driveError = targetDistance - distanceTraveled;
        double driveOutput = this->linearPID.output(driveError);
        double angularError = toNegPos90(targetDeg - to0_360(this->odomSensors->inertial->get_rotation()));
        double angularOutput = this->angularPID.output(angularError);
        driveOutput = limit(driveOutput, -12000 * speedCap, 12000 * speedCap);
        angularOutput = limit(angularOutput, speedCap * 12000, speedCap * (-12000));
        this->setDrive(left_velocity_scaling(driveOutput, angularOutput), right_velocity_scaling(driveOutput, angularOutput));
        prevDistance = distance;
        time += 10;
        pros::delay(10);
    }
    this->setDrive(0, 0);
}

void catlib::Chassis::follow(Curve path, double timeOut, double lookAhead, double speedRatio = 1, bool reversed = 0) {
    Vector2d currentPose = Vector2d(this->getPose()[0], this->getPose()[1]);
    double currentHeading = this->getPoseWithTheta()[2];
    double curvature;
    // Draw a perpendicular line to the connection of control point 2 and target point as the ending line
    double targetDeg = toDeg(atan2(path.controlPoint4[0] - path.controlPoint2[0], path.controlPoint4[1] - path.controlPoint2[1]));
    bool lineSettled = is_line_settled(path.controlPoint4[0], path.controlPoint4[1], targetDeg, this->getPose()[0], this->getPose()[1]);
    double time = 0;
    while ((time * 10 < timeOut && !lineSettled) || path.getCarrotPoint(this->getPose(), lookAhead) != 1) {
        // Check if the robot went through the target line or not
        currentPose = Vector2d(this->getPose()[0], this->getPose()[1]);
        // Check if the robot is settled
        lineSettled = is_line_settled(path.controlPoint4[0], path.controlPoint4[1], targetDeg, this->getPose()[0], this->getPose()[1]);
        currentHeading = this->getPoseWithTheta()[2];
        if (reversed) {
            currentHeading -= 180;
        }
        Vector2d carrotPoint = path.getPoint(path.getCarrotPoint(this->getPose(), lookAhead));

        if (carrotPoint == path.controlPoint4) {
            // Extend the Endline for the Robot to end at the Proper Angle
            Vector2d direction = path.controlPoint4 - path.controlPoint2;
            direction.normalize();
            double d = fabs(lookAhead - (currentPose - path.controlPoint4).norm());
            carrotPoint = d * direction + path.controlPoint4;
        }
        double curvatureHeading = M_PI / 2 - toRadian(currentHeading);
        curvature = curvatureToAPoint(currentPose, curvatureHeading, carrotPoint);
        double angularError = toNegPos180(toDeg(atan2(carrotPoint[0] - this->getPose()[0], carrotPoint[1] - this->getPose()[1])) - to0_360(this->odomSensors->inertial->get_rotation()));
        double curvScaleFactor = cos(toRadian(angularError));
        // Velocity modifiers
        double linearVoltage = 22000 * curvScaleFactor;
        double feedAngularForward = 110000;
        double angularVoltage = feedAngularForward * curvature;
        setDrive(speedRatio * limit_min(left_velocity_scaling(linearVoltage, angularVoltage), 0), speedRatio * limit_min(right_velocity_scaling(linearVoltage, angularVoltage), 0));
        time++;
        pros::delay(10);
    }
    setDrive(0, 0);
}