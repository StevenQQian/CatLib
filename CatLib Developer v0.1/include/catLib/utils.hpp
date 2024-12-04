#pragma once
#include "math.h"

namespace catlib {
    double toRadian(double degree);

    double to0_360(double degree);

    double toNegPos180(double degree);

    double toNegPos90(double degree);
}