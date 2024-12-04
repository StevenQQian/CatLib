#include "main.h"

catlib::PIDConstants::PIDConstants(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

void catlib::PIDConstants::setPIDConstants(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

catlib::PID::PID(PIDConstants* constants, PIDType t) {
    this->constants = constants;
    this->t = t;

    if (this->t == PIDType::ANGULAR) {
        this->integralActivationRange = 5;
    }

    if (this->t == PIDType::LINEAR) {
        this->integralActivationRange = 2;
    }
    
}

catlib::PID::PID() {  
}

void catlib::PID::reset() {
    this->integral = 0;
    this->prevError = 0;
    this->derivative = 0;
}

void catlib::PID::setConstants(double kP, double kI, double kD) {
    this->constants->kP = kP;
    this->constants->kI = kI;
    this->constants->kD = kD;
}

double catlib::PID::output(double error) {
    this->derivative = error - prevError;
    if (fabs(error) < this->integralActivationRange) {
        this->integral += error;
    }
    else {
        this->integral = 0;
    }
    if (error / fabs(error) != this->prevError / fabs(this->prevError)) {
        this->integral = 0;
    }
    double output = (this->constants->kP * error) + (this->constants->kI * integral) + (this->constants->kD * derivative);
    this->prevError = error;
    return output;
}