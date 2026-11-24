#ifndef STEPPER_AXIS_LINEAR_HPP
#define STEPPER_AXIS_LINEAR_HPP

#include "Axis.hpp"
#include "Motor.hpp"

namespace Stepper {
namespace Generic {

class LinearAxis : public Generic::Axis {
public:
    LinearAxis() = delete;
    explicit LinearAxis(Generic::Motor& motor, float distanceMmPerRevolution = 1.0f);
    virtual ~LinearAxis() = default;

    struct LinearAxisTask {
        float position = 0.0f;     // mm (0 = velocity mode)
        float velocity = 0.0f;     // mm/s
        float acceleration = 0.0f; // mm/s^2
        float deceleration = 0.0f; // mm/s^2
        bool moveRelative = true;  // true = relative distance, false = absolute position
    };

    bool run(const LinearAxisTask& task);
    void run(float targetVelocity, float acceleration, float deceleration) override;
    void run(float distance_mm, float targetVelocity, float acceleration, float deceleration) override;

    using Axis::fastStop;
    void fastStop(float deceleration_mm_s2) override;
    void setFastStopDeceleration(float deceleration_mm_s2) override;
    float getFastStopDeceleration() const override;

    float getPosition() const override {
        return getPositionMm();
    };

    void setPosition(float position) override {
        setPositionMm(position);
    };

    float getMinPositionMm() const {
        return getMinLimit();
    };

    float getMaxPositionMm() const {
        return getMaxLimit();
    };

    float getVelocity() const {
        return revolutionsToMm(motor_.getVelocity());
    };

    float getPositionMm() const {
        return revolutionsToMm(motor_.getRevolutions());
    };

    void setPositionMm(float position_mm) {
        motor_.setRevolutions(mmToRevolutions(position_mm));
    };

    float getDistanceMmPerRevolution() const {
        return distanceMmPerRevolution_;
    };

private:
    inline float mmToRevolutions(float distance_mm) const {
        return distance_mm / distanceMmPerRevolution_;
    };

    inline float revolutionsToMm(float revolutions) const {
        return revolutions * distanceMmPerRevolution_;
    };

    float distanceMmPerRevolution_;
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_AXIS_LINEAR_HPP
