#include "Motor.hpp"
#include "Log.hpp"

namespace Stepper {
namespace Generic {

    Motor::Motor(Generic::Generator& generator, uint32_t stepsPerRevolution, float gearRatio)
            : generator_(generator), stepsPerRevolution_(stepsPerRevolution), gearRatio_(gearRatio) {
        stepsPerRevolutionGeared_ = stepsPerRevolution_ * gearRatio_;
        uint8_t microsteps = generator_.driver().getMicrosteps();
        if (microsteps > 1) {
            stepsPerRevolutionGeared_ *= microsteps;
        }
    }

    bool Motor::run(const MotorTask& task) {
        Generic::Generator::GeneratorTask genTask;
        genTask.steps = static_cast<uint32_t>(revolutionsToSteps(fabs(task.revolutions)));
        genTask.velocity = revolutionsToSteps(fabs(task.velocity));
        genTask.acceleration = revolutionsToSteps(fabs(task.acceleration));
        genTask.deceleration = revolutionsToSteps(fabs(task.deceleration));
        genTask.direction = getDirection(task.revolutions, task.velocity);
        return generator_.run(genTask);
    }

    void Motor::run(float targetVelocity, float acceleration, float deceleration) {
        MotorTask task;
        task.revolutions = 0.0f;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        run(task);
    }

    void Motor::run(float revolutions, float targetVelocity, float acceleration, float deceleration) {
        MotorTask task;
        task.revolutions = revolutions;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        run(task);
    }

} // namespace Generic
} // namespace Stepper