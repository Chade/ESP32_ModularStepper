#ifndef STEPPER_AXIS_HPP
#define STEPPER_AXIS_HPP

#include <cstdint>

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
        if (pos == EndstopPosition::Min) return endstopMin_;
        if (pos == EndstopPosition::Max) return endstopMax_;
        return nullptr;
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

    // Homing status flags (managed by Homing module or user)
    bool isHomed() const { return isHomed_; };
    void setHomed(bool homed) { isHomed_ = homed; };
    bool isHoming() const { return isHoming_; };
    void setHoming(bool homing) { isHoming_ = homing; };

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

protected:
    Generic::Motor& motor_;
    Generic::Endstop* endstopMin_ {nullptr};
    Generic::Endstop* endstopMax_ {nullptr};

    bool isHomed_ {false};
    bool isHoming_ {false};
    bool limitProtectionMinEnabled_ {false};
    bool limitProtectionMaxEnabled_ {false};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_AXIS_HPP
