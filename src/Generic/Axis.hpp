#ifndef STEPPER_AXIS_HPP
#define STEPPER_AXIS_HPP

#include <cassert>
#include <cstdint>
#include <limits>

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

    Generic::Endstop* getEndstopMin() const {
        return endstopMin_;
    };

    Generic::Endstop* getEndstopMax() const {
        return endstopMax_;
    };

    Generic::Endstop* getEndstop(EndstopPosition pos) const {
        if (pos == EndstopPosition::Min)
            return endstopMin_;
        if (pos == EndstopPosition::Max)
            return endstopMax_;
        return nullptr;
    };

    bool hasEndstopMin() const {
        return endstopMin_ != nullptr;
    };

    bool hasEndstopMax() const {
        return endstopMax_ != nullptr;
    };

    bool hasEndstop(EndstopPosition pos) const {
        return getEndstop(pos) != nullptr;
    };

    // Limit switch protection
    void enableLimitProtectionMin(bool enable = true);
    void enableLimitProtectionMax(bool enable = true);

    bool isLimitProtectionMinEnabled() const {
        return limitProtectionMinEnabled_;
    };

    bool isLimitProtectionMaxEnabled() const {
        return limitProtectionMaxEnabled_;
    };

    bool isDirectionAllowed(Direction direction) const;

    bool isEndstopTriggered(EndstopPosition pos) const {
        Generic::Endstop* endstop = getEndstop(pos);
        return endstop ? endstop->isTriggered() : false;
    };

    // Soft travel limits (in axis units)
    void setLimits(float minPos, float maxPos) {
        assert(minPos < maxPos);
        minLimit_ = minPos;
        maxLimit_ = maxPos;
    };

    void setMinLimit(float minPos) {
        minLimit_ = minPos;
    };

    void setMaxLimit(float maxPos) {
        maxLimit_ = maxPos;
    };

    float getMinLimit() const {
        return minLimit_;
    };

    float getMaxLimit() const {
        return maxLimit_;
    };

    bool isHomed() const {
        return isHomed_;
    };

    void setHomed(bool homed) {
        isHomed_ = homed;
    };

    bool isHoming() const { 
        return isHoming_;
    };

    void setHoming(bool homing) {
        isHoming_ = homing;
    };

    State getState() const {
        return motor_.getState();
    };
    
    Generic::Motor& motor() {
        return motor_;
    };
    
    const Generic::Motor& motor() const {
        return motor_;
    };
    
    // Unit-specific coordinates & motion implemented by derived axis (RotaryAxis, LinearAxis)
    virtual float getPosition() const = 0;
    virtual void setPosition(float position) = 0;
    
    virtual void run(float velocity, float acceleration, float deceleration) = 0;
    virtual void run(float distance, float velocity, float acceleration, float deceleration) = 0;
    
    // Axis motion interface & state
    virtual void stop();
    virtual void emergencyStop();

protected:
    Generic::Motor& motor_;
    Generic::Endstop* endstopMin_ {nullptr};
    Generic::Endstop* endstopMax_ {nullptr};

    float minLimit_ {std::numeric_limits<float>::min()};
    float maxLimit_ {std::numeric_limits<float>::max()};

    bool isHomed_ {false};
    bool isHoming_ {false};
    bool limitProtectionMinEnabled_ {false};
    bool limitProtectionMaxEnabled_ {false};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_AXIS_HPP
