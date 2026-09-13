#include "StepperMotor.hpp"
#include "StepperLog.hpp"

namespace Stepper {
namespace Generic {

    Motor::Motor(Generator& generator, uint32_t stepsPerRevolution, float gearRatio)
        : generator_(generator), stepsPerRevolution_(stepsPerRevolution), gearRatio_(gearRatio) {
        assert(stepsPerRevolution_ > 0);
        assert(gearRatio_ > 0.0f);
        stepsPerRevolutionGeared_ = stepsPerRevolution_ * gearRatio_;
        uint8_t microsteps = generator_.getDriver().getMicrosteps();
        if (microsteps > 1) {
            stepsPerRevolutionGeared_ *= microsteps;
        }
    }

    bool Motor::run(const MotorTask& task) {
        Generator::GeneratorTask genTask;
        genTask.steps = static_cast<uint32_t>(angleDegToSteps(task.angle));
        genTask.velocity = revolutionToSteps(task.velocity);
        genTask.acceleration = revolutionToSteps(task.acceleration);
        genTask.deceleration = revolutionToSteps(task.deceleration);
        genTask.direction = task.direction;
        LOG_INFO("Motor initialized: stepsPerRevolution=%u, gearRatio=%.2f, stepsPerRevolutionGeared=%u", stepsPerRevolution_, gearRatio_, stepsPerRevolutionGeared_);
        LOG_INFO("Steps: %u", genTask.steps);
        return generator_.run(genTask);
    }

    void Motor::run(float targetVelocity, float acceleration, float deceleration, Direction direction) {
        MotorTask task;
        task.angle = 0.0f;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        task.direction = direction;
        run(task);
    }

    void Motor::run(float angle_deg, float targetVelocity, float acceleration, float deceleration, Direction direction) {
        MotorTask task;
        task.angle = angle_deg;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        task.direction = direction;
        run(task);
    }

} // namespace Generic
} // namespace Stepper