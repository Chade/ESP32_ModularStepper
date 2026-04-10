#include "StepperMotor.h"

namespace Stepper {

    Motor::Motor(Generator& generator)
        : generator_(generator) {

    }

    bool Motor::run(const MotorTask& task) {
        Generator::GeneratorTask genTask;
        genTask.steps = static_cast<uint32_t>(angleToSteps(task.angle));
        genTask.velocity = revolutionToSteps(task.velocity);
        genTask.acceleration = revolutionToSteps(task.acceleration);
        genTask.deceleration = revolutionToSteps(task.deceleration);
        genTask.direction = task.direction;
        return generator_.run(genTask);
    }

    void Motor::run(float targetVelocity, float acceleration, float deceleration, Direction direction) {
        MotorTask task;
        task.angle = 0.0;
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

    void Motor::setParams(uint32_t stepsPerRevolution, float gearRatio) {
        stepsPerRevolution_ = stepsPerRevolution;
        gearRatio_ = gearRatio;
    }

    float Motor::getVelocity() const {
        return stepsToRevolution(generator_.getVelocity());
    }

    float Motor::getAngle() const {
        return stepsToAngle(generator_.getStepsDone());
    }

    State Motor::getState() const {
        return generator_.getState();
    }

    Generator& Motor::getGenerator() {
        return generator_;
    }

} // namespace Stepper