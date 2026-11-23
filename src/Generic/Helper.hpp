#ifndef STEPPER_HELPER_HPP
#define STEPPER_HELPER_HPP

#include <cstdint>
#include <FixedPoints.h>
#include <type_traits>

using UQ20x12 = UFixed<20, 12>;

namespace Stepper {

template <typename T>
constexpr auto cast_enum_to_base(T e)
{
    return static_cast<std::underlying_type_t<T>>(e);
};

template <typename T>
constexpr std::underlying_type_t<T> compare_enums(T e1, T e2)
{
    return cast_enum_to_base(e1) & cast_enum_to_base(e2);
};

template <typename T1, typename T2>
constexpr bool compare_enums(T1 e1, T2 e2)
{
    return static_cast<int64_t>(cast_enum_to_base(e1)) == static_cast<int64_t>(cast_enum_to_base(e2));
};

template <typename T>
constexpr int sign(T val) {
    return (T(0) <=  val) ? 1 : -1;
};

template <typename T> 
constexpr int signum(T val) {
    return (T(0) < val) - (val < T(0));
};

enum class Direction : int8_t
{
    Negative = -1,
    Down = -1,
    Backward = -1,
    Clockwise = -1,
    Neutral = 0,
    Counterclockwise = 1,
    Forward = 1,
    Up = 1,
    Positive = 1
};

inline constexpr Direction reverseDirection(Direction dir) {
    return static_cast<Direction>(-cast_enum_to_base(dir));
};

template <typename T>
constexpr Direction getDirection(T v) {
    return static_cast<Direction>(sign(v));
};

template <typename T1, typename T2>
constexpr Direction getDirection(T1 v1, T2 v2) {
    return static_cast<Direction>(sign(v1) * sign(v2));
};

enum class State : uint8_t
{
    Undefined = 0,
    Manual = 1,
    Running = 2,
    Accelerating = 6,
    Decelerating = 10,
    Paused = 16,
    Stopped = 32,
    Inhibited = 96,
    EmergencyStop = 224
};

// Fast integer square root for 64-bit values (floor(sqrt(val)))
inline constexpr uint64_t isqrt64(uint64_t val) {
    if (val == 0) return 0;

    // Skip leading zero-pair iterations using Count Leading Zeros
    int shift = (63 - __builtin_clzll(val)) & ~1;
    uint64_t bit = 1ULL << shift;
    uint64_t root = 0;

    while (bit != 0) {
        if (val >= root + bit) {
            val -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }

    return root;
};

// Computes the square root of a UQ20x12 fixed-point value cleanly
inline constexpr UQ20x12 sqrt(UQ20x12 val) {
    constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
    uint64_t val_raw = static_cast<uint64_t>(val.getInternal());

    if (val_raw == 0) {
        return UQ20x12::fromInternal(0);
    }

    uint64_t root_raw = isqrt64(val_raw * scale);

    return UQ20x12::fromInternal(root_raw);
};

} // namespace Stepper

#endif // STEPPER_HELPER_HPP