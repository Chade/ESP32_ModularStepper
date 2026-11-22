#ifndef STEPPER_AXIS_HPP
#define STEPPER_AXIS_HPP

#include <cstdint>
#include <functional>
#include <chrono>

#include "Motor.hpp"
#include "Endstop.hpp"
#include "Helper.hpp"
#include "Log.hpp"

namespace Stepper {
namespace Generic {

class Axis {
public:
    enum class EndstopPosition : int8_t {
        Min = -1,
        None = 0,
        Max = 1
    };

    enum class HomingState : uint8_t {
        Idle = 0,
        FastApproach = 1,
        Retracting = 2,
        SlowApproach = 3,
        Done = 4,
        Failed = 5
    };

    struct HomingConfig {
        EndstopPosition endstop = EndstopPosition::None; // Endstop to use for homing (Min or Max)
        Direction direction = Direction::Neutral; // Neutral = auto based on endstop: Min -> CCW, Max -> CW
        float fastVelocity = 50.0f;       // in axis units/s
        float slowVelocity = 10.0f;       // latch velocity in axis units/s
        float acceleration = 100.0f;      // in axis units/s^2
        float deceleration = 100.0f;      // in axis units/s^2
        float retractDistance = 5.0f;     // distance to back off in axis units
        float homePosition = 0.0f;        // coordinate assigned when homed
        uint32_t timeoutMs = 15000;       // max time in ms (0 = disabled)
    };

    using YieldFn = std::function<void(uint32_t delayMs)>;
    using TimeFn = std::function<uint32_t()>;

    Axis() = delete;
    explicit Axis(Generic::Motor& motor);
    virtual ~Axis() = default;

    // Endstop attachment
    void setEndstopMin(Generic::Endstop* endstop, bool enableLimitProtection = true);
    void setEndstopMin(Generic::Endstop& endstop, bool enableLimitProtection = true);
    void setEndstopMax(Generic::Endstop* endstop, bool enableLimitProtection = true);
    void setEndstopMax(Generic::Endstop& endstop, bool enableLimitProtection = true);
    void setEndstops(Generic::Endstop* minEndstop, Generic::Endstop* maxEndstop, bool enableLimitProtectionMin = true, bool enableLimitProtectionMax = true);
    void setEndstop(EndstopPosition pos, Generic::Endstop* endstop, bool enableLimitProtection = true);

    Generic::Endstop* getEndstopMin() const { return endstopMin_; };
    Generic::Endstop* getEndstopMax() const { return endstopMax_; };
    Generic::Endstop* getEndstop(EndstopPosition pos) const {
        return (pos == EndstopPosition::Min) ? endstopMin_ : endstopMax_;
    };

    bool hasEndstopMin() const { return endstopMin_ != nullptr; };
    bool hasEndstopMax() const { return endstopMax_ != nullptr; };
    bool hasEndstop(EndstopPosition pos) const { return getEndstop(pos) != nullptr; };

    // Limit switch protection
    void enableLimitProtectionMin(bool enable = true);
    void enableLimitProtectionMax(bool enable = true);
    bool isLimitProtectionMinEnabled() const { return limitProtectionMinEnabled_; };
    bool isLimitProtectionMaxEnabled() const { return limitProtectionMaxEnabled_; };

    bool isEndstopTriggered(EndstopPosition pos) const {
        Generic::Endstop* e = getEndstop(pos);
        return e ? e->isTriggered() : false;
    };

    // Homing settings & state
    void setHomingConfig(const HomingConfig& config) { homingConfig_ = config; };
    const HomingConfig& getHomingConfig() const { return homingConfig_; };
    HomingConfig& getHomingConfig() { return homingConfig_; };

    bool isHomed() const { return isHomed_; };
    void setHomed(bool homed) { isHomed_ = homed; };
    bool isHoming() const { return isHoming_; };
    HomingState getHomingState() const { return homingState_; };

    // Synchronous two-stage homing routines
    virtual bool home(const HomingConfig& config);
    bool home(EndstopPosition pos = EndstopPosition::Min);
    bool homeMin();
    bool homeMax();

    // Axis motion interface & state
    virtual void stop();
    virtual void emergencyStop();
    virtual State getState() const { return motor_.getState(); };

    Generic::Motor& motor() { return motor_; };
    const Generic::Motor& motor() const { return motor_; };

    // Unit-specific coordinates & motion implemented by derived axis (RotaryAxis, LinearAxis)
    virtual float getPosition() const = 0;
    virtual void setPosition(float position) = 0;

    virtual void run(float velocity, float acceleration, float deceleration, Direction direction) = 0;
    virtual void run(float distance, float velocity, float acceleration, float deceleration, Direction direction) = 0;

    // Custom time & yield hooks (for host sandbox testing vs RTOS)
    void setYieldFunction(YieldFn fn) { yieldFn_ = std::move(fn); };
    void setTimeFunction(TimeFn fn) { timeFn_ = std::move(fn); };

protected:
    uint32_t getTimeMs() const;
    void yieldMs(uint32_t delayMs) const;

    Generic::Motor& motor_;
    Generic::Endstop* endstopMin_ {nullptr};
    Generic::Endstop* endstopMax_ {nullptr};

    HomingConfig homingConfig_;
    HomingState homingState_ {HomingState::Idle};
    bool isHomed_ {false};
    bool isHoming_ {false};
    bool limitProtectionMinEnabled_ {false};
    bool limitProtectionMaxEnabled_ {false};

    YieldFn yieldFn_ {nullptr};
    TimeFn timeFn_ {nullptr};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_AXIS_HPP

