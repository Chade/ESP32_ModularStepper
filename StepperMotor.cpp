#include "StepperMotor.hpp"
#include "StepperLog.hpp"
#include <cassert>

namespace Stepper {
namespace Generic {

    Motor::Motor(Generic::Generator& generator, uint32_t stepsPerRevolution, float gearRatio)
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
        Generic::Generator::GeneratorTask genTask;
        genTask.steps = static_cast<uint32_t>(revolutionsToSteps(task.revolutions));
        genTask.velocity = revolutionsToSteps(task.velocity);
        genTask.acceleration = revolutionsToSteps(task.acceleration);
        genTask.deceleration = revolutionsToSteps(task.deceleration);
        genTask.direction = task.direction;
        return generator_.run(genTask);
    }

    void Motor::run(float targetVelocity, float acceleration, float deceleration, Direction direction) {
        MotorTask task;
        task.revolutions = 0.0f;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        task.direction = direction;
        run(task);
    }

    void Motor::run(float revolutions, float targetVelocity, float acceleration, float deceleration, Direction direction) {
        MotorTask task;
        task.revolutions = revolutions;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        task.direction = direction;
        run(task);
    }

} // namespace Generic
} // namespace Stepper