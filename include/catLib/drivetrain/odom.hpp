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

        pros::Rotation* rotation = nullptr;
        double wheelDiameter;
        double offset;
};

class OdomSensors {
    public:
        OdomSensors(pros::Imu* inertial, TrackingWheel* vertical, TrackingWheel* horizontal);

        TrackingWheel* vertical = nullptr;
        TrackingWheel* horizontal = nullptr;
        pros::Imu* inertial = nullptr;
};
} // namespace catlib