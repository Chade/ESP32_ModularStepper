#ifndef STEPPER_HELPER_H
#define STEPPER_HELPER_H

#include <type_traits>
#include <cstdint>
#include <esp_attr.h>

namespace Stepper
{
    template <typename T>
    auto IRAM_ATTR cast_enum_to_base(T e)
    {
        return static_cast<std::underlying_type_t<T>>(e);
    };

    template <typename T>
    std::underlying_type_t<T> IRAM_ATTR compare_enums(T e1, T e2)
    {
        return cast_enum_to_base(e1) & cast_enum_to_base(e2);
    };

    enum class Direction : int8_t
    {
        Clockwise = -1,
        Neutral = 0,
        Counterclockwise = 1
    };

    enum class State : uint8_t
    {
        Undefined = 0,
        Running = 1,
        Accelerating = 3,
        Decelerating = 5,
        Paused = 16,
        Stopped = 32,
        Inhibited = 96,
        EmergencyStop = 224
    };
}

#endif // STEPPER_HELPER_H