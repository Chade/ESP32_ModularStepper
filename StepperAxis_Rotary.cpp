#include "StepperAxis_Rotary.hpp"
#include "StepperLog.hpp"
#include <cassert>

namespace Stepper {
namespace Generic {

RotaryAxis::RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution, bool continuous)
    : Generic::Axis(motor), angleDegPerRevolution_(angleDegPerRevolution), continuous_(continuous) {
    assert(angleDegPerRevolution_ > 0);

    // Setup rotary-specific default homing parameters (degrees & deg/s)
    homingConfig_.fastVelocity = 60.0f;       // 60 deg/s
    homingConfig_.slowVelocity = 10.0f;       // 10 deg/s
    homingConfig_.acceleration = 120.0f;      // 120 deg/s^2
    homingConfig_.deceleration = 120.0f;      // 120 deg/s^2
    homingConfig_.retractDistance = 10.0f;    // 10 deg back-off
    homingConfig_.homePosition = 0.0f;
    homingConfig_.timeoutMs = 15000;
}

bool RotaryAxis::run(const RotaryAxisTask& task) {
    float angle_deg = task.angle;
    if (!continuous_ && !isHoming_) {
        if (angle_deg < minAngleDeg_) {
            LOG_WARN("Angle %.2f deg is below minimum angle %.2f deg", angle_deg, minAngleDeg_);
            angle_deg = minAngleDeg_;
        }
        else if (angle_deg > maxAngleDeg_) {
            LOG_WARN("Angle %.2f deg is above maximum angle %.2f deg", angle_deg, maxAngleDeg_);
            angle_deg = maxAngleDeg_;
        }
    }

    Generic::Motor::MotorTask motorTask;
    motorTask.revolutions = angleDegToRevolutions(angle_deg);
    motorTask.velocity = angleDegToRevolutions(task.velocity);
    motorTask.acceleration = angleDegToRevolutions(task.acceleration);
    motorTask.deceleration = angleDegToRevolutions(task.deceleration);
    motorTask.direction = task.direction;
    return motor_.run(motorTask);
}

void RotaryAxis::run(float targetVelocity, float acceleration, float deceleration, Direction direction) {
    RotaryAxisTask task;
    task.angle = 0.0f;
    task.velocity = targetVelocity;
    task.acceleration = acceleration;
    task.deceleration = deceleration;
    task.direction = direction;
    run(task);
}

void RotaryAxis::run(float angle_deg, float targetVelocity, float acceleration, float deceleration, Direction direction) {
    RotaryAxisTask task;
    task.angle = angle_deg;
    task.velocity = targetVelocity;
    task.acceleration = acceleration;
    task.deceleration = deceleration;
    task.direction = direction;
    run(task);
}

} // namespace Generic
} // namespace Stepper