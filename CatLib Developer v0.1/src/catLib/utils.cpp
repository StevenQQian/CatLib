#include "main.h"

double catlib::toRadian(double degree) {
    return degree / 180 * M_PI;
}

double catlib::to0_360(double degree) {
    while (degree < 0 || degree > 360) {
        if (degree > 360) degree -= 360;
        else if (degree < 0) degree += 360;
    }
    return degree;
}

double catlib::toNegPos180(double degree) {
    while (degree < -180 || degree > 180) {
        if (degree > 180) degree -= 360;
        else if (degree < -180) degree += 360;
    }
    return degree;
}

double catlib::toNegPos90(double degree) {
    while (degree < -90 || degree > 90) {
        if (degree > 90) degree -= 180;
        else if (degree < -90) degree += 180;
    }
    return degree;
}