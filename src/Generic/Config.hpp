#ifndef STEPPER_CONFIG_HPP
#define STEPPER_CONFIG_HPP

#include <cstdint>

#include "Pin.hpp"

namespace Stepper {
namespace Generic {

struct PinConfig {
    uint8_t gpio {0};
    bool inverse {false};
    Pin::IoMode ioMode {Pin::IoMode::Input_Output};
    Pin::PullMode pullMode {Pin::PullMode::Floating};
};

struct DriverConfig {
    enum class Type : uint8_t {
        MCPWM = 0
    };

    Type type {Type::MCPWM};

    PinConfig enablePin {0, false, Pin::IoMode::Output, Pin::PullMode::Floating};
    PinConfig stepPin {0, false, Pin::IoMode::Output, Pin::PullMode::Floating};
    PinConfig directionPin {0, false, Pin::IoMode::Output, Pin::PullMode::Floating};

    uint8_t microsteps {1};

    float minPulseWidthHigh_us {1.0f};
    float minPulseWidthLow_us {1.0f};
    float directionDelay_us {0.2f};
    float enableDelay_us {0.2f};
};

struct MotorConfig {
    DriverConfig driver;

    uint32_t stepsPerRevolution {200};
    float gearRatio {1.0f};
    float fastStopDeceleration {0.0f}; // rev/s^2 (0 = fall back to active task deceleration)
};

struct EndstopConfig {
    bool enabled {false};
    PinConfig pin {0, true, Pin::IoMode::Input, Pin::PullMode::PullUp};
    uint32_t debounceUs {25000};
    bool limitProtection {true};
};

struct AxisConfig {
    MotorConfig motor;

    EndstopConfig endstopMin;
    EndstopConfig endstopMax;

    bool hasSoftLimits {false};
    float minLimit {0.0f};
    float maxLimit {0.0f};
    float fastStopDeceleration {0.0f}; // axis units/s^2 (0 = keep MotorConfig / task deceleration)
};

struct LinearAxisConfig : public AxisConfig {
    float distanceMmPerRevolution {1.0f};
};

struct RotaryAxisConfig : public AxisConfig {
    float angleDegPerRevolution {360.0f};
    bool continuous {true};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_CONFIG_HPP
