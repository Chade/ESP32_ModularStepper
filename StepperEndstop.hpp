#ifndef STEPPER_ENDSTOP_HPP
#define STEPPER_ENDSTOP_HPP

#include "StepperPin.hpp"

namespace Stepper {
namespace Generic {
class Endstop {
public:
    Endstop() = delete;
    Endstop(Pin& pin)
        : pin_(pin) {};

    virtual ~Endstop() = default;

protected:
    Pin& pin_;
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_ENDSTOP_HPP