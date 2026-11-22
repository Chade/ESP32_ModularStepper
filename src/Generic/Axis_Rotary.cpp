#include "Axis_Rotary.hpp"
#include "Log.hpp"
#include <cassert>

namespace Stepper {
namespace Generic {

RotaryAxis::RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution, bool continuous)
    : Generic::Axis(motor), angleDegPerRevolution_(angleDegPerRevolution), continuous_(continuous) {
    assert(angleDegPerRevolution_ > 0);
    setLimits(0.0f, 360.0f);
}

bool RotaryAxis::run(const RotaryAxisTask& task) {
    if (!isDirectionAllowed(task.direction)) {
        return false;
    }

    float angle_deg = task.angle;
    if (!continuous_ && !isHoming_) {
        if (angle_deg < minLimit_) {
            LOG_WARN("Angle %.2f deg is below minimum angle %.2f deg", angle_deg, minLimit_);
            angle_deg = minLimit_;
        }
        else if (angle_deg > maxLimit_) {
            LOG_WARN("Angle %.2f deg is above maximum angle %.2f deg", angle_deg, maxLimit_);
            angle_deg = maxLimit_;
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