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
    if (!isDirectionAllowed(getDirection(task.angle, task.velocity))) {
        return false;
    }

    if (!task.moveRelative && !isHomed_) {
        LOG_WARN("Cannot move to absolute position because axis is not homed");
        return false;
    }

    float angle;
    float targetAngle;
    float currentAngle = getPosition();
    if (task.moveRelative) {
        angle = task.angle;
        targetAngle = currentAngle + angle;
    }
    else {
        targetAngle = task.angle;
        angle = targetAngle - currentAngle;
    }

    if (angle != 0.0f && !isHoming_  && isHomed_ && !continuous_) {
        if (targetAngle < minLimit_) {
            LOG_WARN("Target angle %.2f deg is below minimum angle %.2f deg", targetAngle, minLimit_);
            angle = std::max(0.0f, currentAngle - minLimit_);
        }
        else if (targetAngle > maxLimit_) {
            LOG_WARN("Target angle %.2f deg is above maximum angle %.2f deg", targetAngle, maxLimit_);
            angle = std::max(0.0f, maxLimit_ - currentAngle);
        }
    }

    Generic::Motor::MotorTask motorTask;
    motorTask.revolutions = angleDegToRevolutions(angle);
    motorTask.velocity = angleDegToRevolutions(task.velocity);
    motorTask.acceleration = angleDegToRevolutions(task.acceleration);
    motorTask.deceleration = angleDegToRevolutions(task.deceleration);
    return motor_.run(motorTask);
}

void RotaryAxis::run(float targetVelocity, float acceleration, float deceleration) {
    RotaryAxisTask task;
    task.angle = 0.0f;
    task.velocity = targetVelocity;
    task.acceleration = acceleration;
    task.deceleration = deceleration;
    run(task);
}

void RotaryAxis::run(float angle_deg, float targetVelocity, float acceleration, float deceleration) {
    RotaryAxisTask task;
    task.angle = angle_deg;
    task.velocity = targetVelocity;
    task.acceleration = acceleration;
    task.deceleration = deceleration;
    run(task);
}

} // namespace Generic
} // namespace Stepper