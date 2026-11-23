#ifndef STEPPER_AXIS_ROTARY_HPP
#define STEPPER_AXIS_ROTARY_HPP

#include <cassert>

#include "Axis.hpp"
#include "Motor.hpp"

namespace Stepper {
namespace Generic {

class RotaryAxis : public Generic::Axis {
public:
    RotaryAxis() = delete;
    explicit RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution = 360.0f, bool continuous = true);
    virtual ~RotaryAxis() = default;

    struct RotaryAxisTask {
        float angle  = 0.0f;        // degrees (0 = velocity mode)
        float velocity = 0.0f;      // degrees/s
        float acceleration = 0.0f;  // degrees/s^2
        float deceleration = 0.0f;  // degrees/s^2
        bool moveRelative = true;   // true = relative angle, false = absolute position
    };

    bool run(const RotaryAxisTask& task);
    void run(float targetVelocity, float acceleration, float deceleration) override;
    void run(float angle_deg, float targetVelocity, float acceleration, float deceleration) override;

    float getPosition() const override {
        return getAngleDeg();
    };

    void setPosition(float position) override {
        setAngleDeg(position);
    };

    float getMinAngleDeg() const {
        return getMinLimit();
    };

    float getMaxAngleDeg() const {
        return getMaxLimit();
    };

    float getVelocity() const {
        return revolutionsToAngleDeg(motor_.getVelocity());
    };

    float getAngleDeg() const {
        return revolutionsToAngleDeg(motor_.getRevolutions());
    };

    void setAngleDeg(float angle_deg) {
        motor_.setRevolutions(angleDegToRevolutions(angle_deg));
    };

    bool isContinuous() const {
        return continuous_;
    };

private:
    inline float angleDegToRevolutions(float angle_deg) const {
        return angle_deg / angleDegPerRevolution_;
    };

    inline float revolutionsToAngleDeg(float revolutions) const {
        return revolutions * angleDegPerRevolution_;
    };

    float angleDegPerRevolution_;
    bool continuous_;
};

} // namespace Generic

namespace Axis {
    using RotaryAxis = Generic::RotaryAxis;
} // namespace Axis
} // namespace Stepper

#endif // STEPPER_AXIS_ROTARY_HPP