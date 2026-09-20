#ifndef STEPPER_AXIS_ROTARY_HPP
#define STEPPER_AXIS_ROTARY_HPP

#include "StepperMotor.hpp"

namespace Stepper {
namespace Axis {

class RotaryAxis {
public:
    RotaryAxis() = delete;
    RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution = 360.0f, bool continuous = true);
    virtual ~RotaryAxis() = default;

    struct RotaryAxisTask {
        float angle  = 0.0;
        float velocity = 0.0;
        float acceleration = 0.0;
        float deceleration = 0.0;
        Direction direction = Direction::Neutral;
    };

    bool run(const RotaryAxisTask& task);
    void run(float targetVelocity, float acceleration, float deceleration, Direction direction);
    void run(float angle_deg, float targetVelocity, float acceleration, float deceleration, Direction direction);

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
        return revolutionsToAngleDeg(motor_.getRevolutions());
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

    Generic::Motor& motor_;
    float angleDegPerRevolution_;
    bool continuous_;
    float minAngleDeg_ {0.0f};
    float maxAngleDeg_ {360.0f};
    
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_AXIS_ROTARY_HPP