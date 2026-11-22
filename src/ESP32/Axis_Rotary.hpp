#ifndef STEPPER_AXIS_ROTARY_ESP32_HPP
#define STEPPER_AXIS_ROTARY_ESP32_HPP

#include "../Generic/Axis_Rotary.hpp"
#include "Axis.hpp"

namespace Stepper {
namespace ESP32 {

class RotaryAxis : public Generic::RotaryAxis, public ESP32::Axis {
public:
    RotaryAxis() = delete;
    RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution = 360.0f, bool continuous = true);
    ~RotaryAxis() override = default;
};

} // namespace ESP32
} // namespace Stepper

#endif // STEPPER_AXIS_ROTARY_ESP32_HPP
