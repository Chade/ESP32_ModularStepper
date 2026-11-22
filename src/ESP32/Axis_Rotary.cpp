#include "Axis_Rotary.hpp"

namespace Stepper {
namespace ESP32 {

RotaryAxis::RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution, bool continuous)
    : Generic::Axis(motor),
      Generic::RotaryAxis(motor, angleDegPerRevolution, continuous),
      ESP32::Axis(motor) {
}

} // namespace ESP32
} // namespace Stepper
