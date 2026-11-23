#include <algorithm>
#include <cassert>

#include "Axis_Linear.hpp"
#include "Log.hpp"

namespace Stepper {
namespace Generic {

LinearAxis::LinearAxis(Generic::Motor& motor, float distanceMmPerRevolution)
    : Generic::Axis(motor), distanceMmPerRevolution_(distanceMmPerRevolution) {
    assert(distanceMmPerRevolution_ > 0.0f);
}

bool LinearAxis::run(const LinearAxisTask& task) {
    if (!task.moveRelative && !isHomed_) {
        LOG_WARN("Cannot move to absolute position because axis is not homed");
        return false;
    }

    float distance = 0.0f;
    float velocity = task.velocity;
    float currentPos = getPosition();
    float targetPos = currentPos;

    if (task.moveRelative) {
        if (task.position == 0.0f) {
            return true;
        }
        distance = task.position * static_cast<float>(sign(task.velocity));
        velocity = fabs(task.velocity);
        targetPos = currentPos + distance;
    }
    else {
        if (task.position == currentPos) {
            return true;
        }
        targetPos = task.position;
        distance = targetPos - currentPos;
        velocity = fabs(task.velocity);
    }

    if (!isDirectionAllowed(getDirection(distance, velocity))) {
        return false;
    }

    if (distance != 0.0f && !isHoming_ && isHomed_) {
        if (targetPos < minLimit_) {
            LOG_WARN("Target position %.2f mm is below minimum limit %.2f mm", targetPos, minLimit_);
            distance = std::min(0.0f, minLimit_ - currentPos);
            if (distance == 0.0f) {
                return false;
            }
        }
        else if (targetPos > maxLimit_) {
            LOG_WARN("Target position %.2f mm is above maximum limit %.2f mm", targetPos, maxLimit_);
            distance = std::max(0.0f, maxLimit_ - currentPos);
            if (distance == 0.0f) {
                return false;
            }
        }
    }

    Generic::Motor::MotorTask motorTask;
    motorTask.revolutions = mmToRevolutions(distance);
    motorTask.velocity = mmToRevolutions(velocity);
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
