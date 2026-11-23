#include "Axis_Linear.hpp"
#include "Log.hpp"
#include <algorithm>
#include <cassert>

namespace Stepper {
namespace Generic {

LinearAxis::LinearAxis(Generic::Motor& motor, float distanceMmPerRevolution)
    : Generic::Axis(motor), distanceMmPerRevolution_(distanceMmPerRevolution) {
    assert(distanceMmPerRevolution_ > 0.0f);
}

bool LinearAxis::run(const LinearAxisTask& task) {
    if (!isDirectionAllowed(getDirection(task.position, task.velocity))) {
        return false;
    }

    if (!task.moveRelative && !isHomed_) {
        LOG_WARN("Cannot move to absolute position because axis is not homed");
        return false;
    }

    float position;
    float targetPos;
    float currentPos = getPosition();
    if (task.moveRelative) {
        position = task.position;
        targetPos = currentPos + position;
    }
    else {
        targetPos = task.position;
        position = targetPos - currentPos;
    }

    if (position != 0.0f && !isHoming_  && isHomed_) {
        if (targetPos < minLimit_) {
            LOG_WARN("Target position %.2f mm is below minimum limit %.2f mm", targetPos, minLimit_);
            position = std::max(0.0f, currentPos - minLimit_);
        }
        else if (targetPos > maxLimit_) {
            LOG_WARN("Target position %.2f mm is above maximum limit %.2f mm", targetPos, maxLimit_);
            position = std::max(0.0f, maxLimit_ - currentPos);
        }
    }

    Generic::Motor::MotorTask motorTask;
    motorTask.revolutions = mmToRevolutions(position);
    motorTask.velocity = mmToRevolutions(task.velocity);
    motorTask.acceleration = mmToRevolutions(task.acceleration);
    motorTask.deceleration = mmToRevolutions(task.deceleration);
    return motor_.run(motorTask);
}

void LinearAxis::run(float targetVelocity, float acceleration, float deceleration) {
    LinearAxisTask task;
    task.position = 0.0f;
    task.velocity = targetVelocity;
    task.acceleration = acceleration;
    task.deceleration = deceleration;
    run(task);
}

void LinearAxis::run(float distance_mm, float targetVelocity, float acceleration, float deceleration) {
    LinearAxisTask task;
    task.position = distance_mm;
    task.velocity = targetVelocity;
    task.acceleration = acceleration;
    task.deceleration = deceleration;
    run(task);
}

} // namespace Generic
} // namespace Stepper
