#ifndef STEPPER_HOMING_HPP
#define STEPPER_HOMING_HPP

#include <cstdint>
#include <functional>

#include "Axis.hpp"

namespace Stepper {
namespace Generic {

class Homing {
public:
    enum class State : uint8_t {
        Idle = 0,
        FastApproach = 1,
        Retracting = 2,
        SlowApproach = 3,
        BackingOff = 4,
        Done = 5,
        Failed = 6
    };

    struct Config {
        Axis::EndstopPosition endstop = Axis::EndstopPosition::Min;
        Direction direction = Direction::Neutral; // Neutral = auto based on endstop: Min -> CCW, Max -> CW
        float fastVelocity = 50.0f;       // in axis units/s
        float slowVelocity = 10.0f;       // latch velocity in axis units/s
        float acceleration = 100.0f;      // in axis units/s^2
        float deceleration = 100.0f;      // in axis units/s^2
        float retractDistance = 5.0f;     // distance to back off between fast and slow approach in axis units
        float backoffDistance = 0.0f;     // distance to pull off the switch after final latch in axis units (0 = stay on switch)
        float homePosition = 0.0f;        // coordinate assigned at final home position
        uint32_t timeoutMs = 15000;       // max time in ms (0 = disabled)
    };

    using YieldFn = std::function<void(uint32_t delayMs)>;
    using TimeFn = std::function<uint32_t()>;

    Homing() = default;
    explicit Homing(const Config& defaultConfig) : config_(defaultConfig) {};
    virtual ~Homing() = default;

    void setConfig(const Config& config) { config_ = config; };
    const Config& getConfig() const { return config_; };
    Config& getConfig() { return config_; };

    State getState() const { return state_; };
    bool isHoming() const {
        return state_ == State::FastApproach ||
               state_ == State::Retracting ||
               state_ == State::SlowApproach ||
               state_ == State::BackingOff;
    };

    // Synchronous two-stage homing routines
    virtual bool home(Axis& axis, const Config& config);
    bool home(Axis& axis, Axis::EndstopPosition pos = Axis::EndstopPosition::Min);
    bool homeMin(Axis& axis);
    bool homeMax(Axis& axis);

    // Custom time & yield hooks (for host sandbox testing vs RTOS)
    void setYieldFunction(YieldFn fn) { yieldFn_ = std::move(fn); };
    void setTimeFunction(TimeFn fn) { timeFn_ = std::move(fn); };

protected:
    uint32_t getTimeMs() const;
    void yieldMs(uint32_t delayMs) const;
    static void restoreLimitProtection(Axis& axis, Axis::EndstopPosition pos);

    Config config_;
    State state_ {State::Idle};

    YieldFn yieldFn_ {nullptr};
    TimeFn timeFn_ {nullptr};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_HOMING_HPP
