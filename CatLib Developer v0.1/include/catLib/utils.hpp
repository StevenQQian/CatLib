#pragma once
#include "main.h"

namespace catlib {
    double toRadian(double degree) {
        return degree / 180 * M_PI;
    }

    double to0_360(double degree) {
        while (degree < 0 || degree > 360) {
            if (degree > 360) degree -= 360;
            else if (degree < 0) degree += 360;
        }
        return degree;
    }

    double toNegPos180(double degree) {
        while (degree < -180 || degree > 180) {
            if (degree > 180) degree -= 360;
            else if (degree < -180) degree += 360;
        }
        return degree;
    }

    double toNegPos90(double degree) {
        while (degree < -90 || degree > 90) {
            if (degree > 90) degree -= 180;
            else if (degree < -90) degree += 180;
        }
        return degree;
    }
}