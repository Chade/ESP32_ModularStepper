#ifndef STEPPER_AXIS_H
#define STEPPER_AXIS_H

#include "StepperMotor.h"

namespace Stepper {
    class Axis {
    public:
        Axis() = delete;
        Axis(Motor& motor) : motor_(motor) {};
        virtual ~Axis() = default;
    private:
        Motor& motor_;
    };
}

#endif // STEPPER_AXIS_H