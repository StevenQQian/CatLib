#include "catLib/pursuit/curve.hpp"

catlib::Curve::Curve(Vector2d controlPoint1, Vector2d controlPoint2, Vector2d controlPoint3, Vector2d controlPoint4) {
    this->controlPoint1 = controlPoint1;
    this->controlPoint2 = controlPoint2;
    this->controlPoint3 = controlPoint3;
    this->controlPoint4 = controlPoint4;
}

Vector2d catlib::Curve::getPoint(double t) {
    Vector2d curve = pow(1-t, 3)*this->controlPoint1 + 3*pow(1-t, 2)*t*this->controlPoint2 + 3*(1-t)*pow(t, 2)*this->controlPoint3 + pow(t, 3)*this->controlPoint4;
    return curve;
}

double catlib::Curve::getCarrotPoint(Vector2d currentPose, double lookAhead) {
    double minDistance = 1000;
    double minimumT = 0;
    bool notFound = 1;
    double t = 1;
    Vector2d destination = this->getPoint(1);
    double distanceToDestination = (destination - currentPose).norm();
    if (distanceToDestination <= lookAhead) {
        return 1;
    }
    while (t >= 0) {
        Vector2d pose = this->getPoint(t);
        double distance = (pose - currentPose).norm();
        if (distance < minDistance) {
            minDistance = distance;
            minimumT = t;
        }
        if (distance <= lookAhead) {
            return t;
        }
        if (t >= 0) {
            t -= 0.01;
        }
    }
    return minimumT;
}