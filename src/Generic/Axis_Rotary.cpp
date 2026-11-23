#include <cassert>

#include "Axis_Rotary.hpp"
#include "Log.hpp"

namespace Stepper {
namespace Generic {

RotaryAxis::RotaryAxis(Generic::Motor& motor, float angleDegPerRevolution, bool continuous)
        : Generic::Axis(motor), angleDegPerRevolution_(angleDegPerRevolution), continuous_(continuous) {
    assert(angleDegPerRevolution_ > 0);
    setLimits(0.0f, 360.0f);
}

bool RotaryAxis::run(const RotaryAxisTask& task) {
    if (!task.moveRelative && !isHomed_) {
        LOG_WARN("Cannot move to absolute position because axis is not homed");
        return false;
    }

    float angle = 0.0f;
    float velocity = task.velocity;
    float currentAngle = getPosition();
    float targetAngle = currentAngle;

    if (task.moveRelative) {
        if (task.angle != 0.0f) {
            angle = task.angle * static_cast<float>(sign(task.velocity));
            velocity = fabs(task.velocity);
            targetAngle = currentAngle + angle;
        }
    }
    else {
        targetAngle = task.angle;
        angle = targetAngle - currentAngle;
        velocity = fabs(task.velocity);
    }

    if (!isDirectionAllowed(getDirection(angle, velocity))) {
        return false;
    }

    if (angle != 0.0f && !isHoming_ && isHomed_ && !continuous_) {
        if (targetAngle < minLimit_) {
            LOG_WARN("Target angle %.2f deg is below minimum angle %.2f deg", targetAngle, minLimit_);
            angle = std::min(0.0f, minLimit_ - currentAngle);
            if (angle == 0.0f) {
                return false;
            }
        }
        else if (targetAngle > maxLimit_) {
            LOG_WARN("Target angle %.2f deg is above maximum angle %.2f deg", targetAngle, maxLimit_);
            angle = std::max(0.0f, maxLimit_ - currentAngle);
            if (angle == 0.0f) {
                return false;
            }
        }
    }

    Generic::Motor::MotorTask motorTask;
    motorTask.revolutions = angleDegToRevolutions(angle);
    motorTask.velocity = angleDegToRevolutions(velocity);
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