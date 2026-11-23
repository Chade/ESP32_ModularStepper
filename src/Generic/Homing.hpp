#ifndef STEPPER_HOMING_HPP
#define STEPPER_HOMING_HPP

#include <cstdint>
#include <functional>

#include "Axis.hpp"

namespace Stepper {
namespace Generic {

class Homing {
public:
    enum class Strategy : int8_t {
        MinThenMax = -2, // Home to Min (minLimit = homePosition), then Max (maxLimit = measured)
        Min = -1,        // Home to Min endstop (sets position = homePosition)
        Max = 1,         // Home to Max endstop (sets position = homePosition)
        MaxThenMin = 2   // Home to Max (maxLimit = homePosition), then Min (minLimit = measured)
    };

    enum class State : uint8_t {
        Idle = 0,
        Done = 1,
        Running = 2,
        FastApproach = 6,
        Retracting = 10,
        SlowApproach = 18,
        BackingOff = 34,
        Failed = 128
    };

    struct Config {
        Strategy strategy = Strategy::Min;
        float fastVelocity = 50.0f;       // in axis units/s
        float slowVelocity = 10.0f;       // latch velocity in axis units/s
        float acceleration = 1000.0f;     // in axis units/s^2
        float deceleration = 1000.0f;     // in axis units/s^2
        float retractDistance = 5.0f;     // distance to back off between fast and slow approach in axis units
        float backoffDistance = 0.0f;     // distance to pull off the switch after final latch in axis units (0 = stay on switch)
        float maxTravelDistance = 0.0f;   // max search distance in axis units before aborting (0 = unlimited)
        float homePosition = 0.0f;        // coordinate assigned at the first latched endstop
    };

    using YieldFn = std::function<void(uint32_t delayMs)>;

    Homing() = default;
    explicit Homing(const Config& defaultConfig) : config_(defaultConfig) {};
    virtual ~Homing() = default;

    void setConfig(const Config& config) {
        config_ = config;
    };

    const Config& getConfig() const {
        return config_;
    };

    Config& getConfig() {
        return config_;
    };

    State getState() const { 
        return state_;
    };

    bool isHoming() const {
        return compare_enums(state_, State::Running);
    };

    bool isHomed() const {
        return compare_enums(state_, State::Done);
    };

    // Synchronous homing routines
    virtual bool home(Axis& axis, const Config& config);
    bool home(Axis& axis);
    bool home(Axis& axis, Strategy strategy);
    bool homeMin(Axis& axis);
    bool homeMax(Axis& axis);
    bool homeMinMax(Axis& axis);
    bool homeMinMax(Axis& axis, const Config& config);
    bool homeMaxMin(Axis& axis);
    bool homeMaxMin(Axis& axis, const Config& config);

    // Custom yield hook (used on host sandbox testing; defaults to vTaskDelay on ESP32)
    void setYieldFunction(YieldFn fn) { yieldFn_ = std::move(fn); };

protected:
    // Reusable single-endstop latch sequence (Step 0 initial retract -> Phase 1 fast -> Phase 2 retract -> Phase 3 slow latch)
    bool homeToEndstop(Axis& axis, Axis::EndstopPosition endstopPos, const Config& config);

    // Reusable post-latch pull-off sequence (Phase 4)
    bool backoffFromEndstop(Axis& axis, Axis::EndstopPosition endstopPos, const Config& config);
    bool backoffFromEndstop(Axis& axis, Axis::EndstopPosition endstopPos, float distance, float velocity, float acceleration, float dceleration);

    static bool isDualEndstopStrategy(Strategy strategy) {
        return strategy == Strategy::MinThenMax || strategy == Strategy::MaxThenMin;
    };

    static Direction searchDirectionFor(Axis::EndstopPosition pos) {
        return static_cast<Direction>(cast_enum_to_base(pos));
    };

    void yieldMs(uint32_t delayMs) const;
    static void restoreLimitProtection(Axis& axis, Axis::EndstopPosition pos);
    void finalizeAxisState(Axis& axis);

    Config config_;
    State state_ {State::Idle};

    YieldFn yieldFn_ {nullptr};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_HOMING_HPP
