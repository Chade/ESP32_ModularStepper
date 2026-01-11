#ifndef STEPPER_HELPER_H
#define STEPPER_HELPER_H

#include <type_traits>

namespace Stepper
{
    template <typename T>
    auto cast_enum_to_base(T e)
    {
        return static_cast<std::underlying_type_t<T>>(e);
    };

    enum class Direction : int8_t
    {
        CLOCKWISE = -1,
        NEUTRAL = 0,
        COUTERCLOCKWISE = 1
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