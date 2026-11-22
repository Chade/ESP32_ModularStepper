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
    RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution = 360.0f, bool continuous = true);
    virtual ~RotaryAxis() = default;

    struct RotaryAxisTask {
        float angle  = 0.0f;
        float velocity = 0.0f;
        float acceleration = 0.0f;
        float deceleration = 0.0f;
        Direction direction = Direction::Neutral;
    };

    bool run(const RotaryAxisTask& task);
    void run(float targetVelocity, float acceleration, float deceleration, Direction direction);
    void run(float angle_deg, float targetVelocity, float acceleration, float deceleration, Direction direction);

    float getPosition() const override {
        return getAngleDeg();
    };

    void setPosition(float position) override {
        setAngleDeg(position);
    };

    void setLimits(float minAngleDeg, float maxAngleDeg) {
        assert(minAngleDeg < maxAngleDeg);
        minAngleDeg_ = minAngleDeg;
        maxAngleDeg_ = maxAngleDeg;
    };

    float getMinAngleDeg() const {
        return minAngleDeg_;
    };

    float getMaxAngleDeg() const {
        return maxAngleDeg_;
    };

    float getVelocity() const {
        return revolutionsToAngleDeg(motor_.getVelocity());
    };

    float getAngleDeg() const {
        return revolutionsToAngleDeg(motor_.getRevolutions()) + angleOffsetDeg_;
    };

    void setAngleDeg(float angle_deg) {
        angleOffsetDeg_ = angle_deg - revolutionsToAngleDeg(motor_.getRevolutions());
    };

    bool isContinuous() const {
        return continuous_;
    };

    float getAngleDegPerRevolution() const {
        return angleDegPerRevolution_;
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
    float minAngleDeg_ {0.0f};
    float maxAngleDeg_ {360.0f};
    float angleOffsetDeg_ {0.0f};
};

} // namespace Generic

namespace Axis {
    using RotaryAxis = Generic::RotaryAxis;
} // namespace Axis
} // namespace Stepper

#endif // STEPPER_AXIS_ROTARY_HPP