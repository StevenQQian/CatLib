#include "main.h"

namespace catlib {
    class PIDConstants {
        public:
            PIDConstants(double kP, double kI, double kD);

            void setPIDConstants(double kP, double kI, double kD);
            double kP;
            double kI;
            double kD;
    };
}