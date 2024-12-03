#pragma once
#include "main.h"

namespace catlib {
    class Curve {
        public:
            Curve(Vector2d controlPoint1, Vector2d controlPoint2, Vector2d controlPoint3, Vector2d ControlPoint4);

            Vector2d getPoint(double t);

            Vector2d getCarrotPoint(Vector2d currentPose, double lookAhead);

        private:
            Vector2d controlPoint1;
            Vector2d controlPoint2;
            Vector2d controlPoint3;
            Vector2d controlPoint4;
    };
}