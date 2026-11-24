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
};

struct EndstopConfig {
    bool enabled {false};
    PinConfig pin {0, true, Pin::IoMode::Input, Pin::PullMode::PullUp};
    uint32_t debounceUs {25000};
    bool limitProtection {true};
};

struct AxisConfig {
    enum class Type : uint8_t {
        Linear = 0,
        Rotary = 1
    };

    Type type {Type::Rotary};
    MotorConfig motor;

    // mm per motor revolution (Linear) or degrees per motor revolution (Rotary)
    float unitsPerRevolution {1.0f};
    // RotaryAxis only: allow continuous rotation without clamping to [min, max]
    bool continuous {true};

    EndstopConfig endstopMin;
    EndstopConfig endstopMax;

    bool hasSoftLimits {false};
    float minLimit {0.0f};
    float maxLimit {0.0f};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_CONFIG_HPP
