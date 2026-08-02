#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

#include "StepperGenerator.hpp"

namespace Stepper {
    class Motor {
    public:
        Motor() = delete;
        Motor(Generator& generator, uint32_t stepsPerRevolution = 200, float gearRatio = 1.0);
        virtual ~Motor() = default;

        struct MotorTask {
            float angle  = 0.0;
            float velocity = 0.0;
            float acceleration = 0.0;
            float deceleration = 0.0;
            Direction direction = Direction::Neutral;
        };

        bool run(const MotorTask& task);
        void run(float targetVelocity, float acceleration, float deceleration, Direction direction);
        void run(float angle_deg, float targetVelocity, float acceleration, float deceleration, Direction direction);

        float angleDegToSteps(float angle_deg) const {
            return (angle_deg / 360.0f) * stepsPerRevolutionGeared_;
        };

        float stepsToAngleDeg(float steps) const {
            return (steps / stepsPerRevolutionGeared_) * 360.0f;
        };

        float revolutionToSteps(float revolution) const {
            return (revolution * stepsPerRevolutionGeared_);
        };

        float stepsToRevolution(float steps) const {
            return (steps / stepsPerRevolutionGeared_);
        };

        uint32_t getStepsPerRevolution() const {
            return stepsPerRevolution_;
        };
        
        float getGearRatio() const {
            return gearRatio_;
        };

        float getVelocity() const {
            return stepsToRevolution(generator_.getVelocity());
        };

        float getAngle() const {
            return stepsToAngleDeg(generator_.getStepsDone());
        };

        State getState() const {
            return generator_.getState();
        };


        Generator& getGenerator() {
            return generator_;
        };

        const Generator& getGenerator() const {
            return generator_;
        };

    private:
        Generator& generator_;
        State state_ = State::Undefined;
        float gearRatio_;
        uint32_t stepsPerRevolution_;
        uint32_t stepsPerRevolutionGeared_;
    };
}

#endif // STEPPER_MOTOR_HPP