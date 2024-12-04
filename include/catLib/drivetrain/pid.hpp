#pragma once

namespace catlib {
    enum class PIDType {
        LINEAR,
        ANGULAR
    };
    class PIDConstants {
        public:
            PIDConstants(double kP, double kI, double kD);

            void setPIDConstants(double kP, double kI, double kD);
            double kP;
            double kI;
            double kD;
    };

    class PID {
        public:
            PID(PIDConstants* constants, PIDType t);

            PID();

            void reset();

            double output(double error);

            void setConstants(double kP, double kI, double kD);

        protected:
            PIDType t;
            double integral;
            double prevError;
            double derivative;
            double integralActivationRange;
            PIDConstants* constants;
    };
}