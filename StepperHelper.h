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
        CLOCKWISE = -1,
        NEUTRAL = 0,
        COUNTERCLOCKWISE = 1
    };

    enum class State : uint8_t
    {
        UNDEFINED = 0,
        RUNNING = 1,
        ACCELERATING = 3,
        DECELERATING = 5,
        PAUSED = 16,
        STOPPED = 32,
        INHIBITED = 96,
        EMERGENCYSTOP = 224
    };
}

#endif // STEPPER_HELPER_H