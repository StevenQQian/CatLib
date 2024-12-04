#pragma once
#include "Eigen/Dense"

namespace catlib {
    class Curve {
        public:
            Curve(Eigen::Vector2d controlPoint1, Eigen::Vector2d controlPoint2, Eigen::Vector2d controlPoint3, Eigen::Vector2d ControlPoint4);

            Eigen::Vector2d getPoint(double t);

            Eigen::Vector2d getCarrotPoint(Eigen::Vector2d currentPose, double lookAhead);

        private:
            Eigen::Vector2d controlPoint1;
            Eigen::Vector2d controlPoint2;
            Eigen::Vector2d controlPoint3;
            Eigen::Vector2d controlPoint4;
    };
}