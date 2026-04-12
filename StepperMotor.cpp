#include "StepperMotor.hpp"
#include "StepperLog.hpp"

namespace Stepper {

    static constexpr const char* log_tag {"Motor"};

    Motor::Motor(Generator& generator)
        : generator_(generator) {

    }

    bool Motor::run(const MotorTask& task) {
        Generator::GeneratorTask genTask;
        genTask.steps = static_cast<uint32_t>(angleDegToSteps(task.angle));
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
        if (stepsPerRevolution == 0) {
            ESP_LOGW(log_tag, "Invalid stepsPerRevolution=0. Keeping previous value: %lu", static_cast<unsigned long>(stepsPerRevolution_));
            return;
        }

        if (gearRatio <= 0.0f) {
            ESP_LOGW(log_tag, "Invalid gearRatio=%f. Keeping previous value: %f", gearRatio, gearRatio_);
            return;
        }

        stepsPerRevolution_ = stepsPerRevolution;
        gearRatio_ = gearRatio;
    }

    float Motor::getVelocity() const {
        return stepsToRevolution(generator_.getVelocity());
    }

    float Motor::getAngle() const {
        return stepsToAngleDeg(generator_.getStepsDone());
    }

    State Motor::getState() const {
        return generator_.getState();
    }

    Generator& Motor::getGenerator() {
        return generator_;
    }

    const Generator& Motor::getGenerator() const {
        return generator_;
    }

} // namespace Stepper