#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

#include "StepperGenerator.hpp"

namespace Stepper {
    class Motor {
    public:
        Motor() = delete;
        Motor(Generator& generator);
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

        void setParams(uint32_t stepsPerRevolution, float gearRatio = 1.0);

        float getVelocity() const;
        float getAngle() const;
        State getState() const;

        float angleDegToSteps(float angle_deg) const {
            return (angle_deg / 360.0f) * stepsPerRevolution_ * gearRatio_;
        };

        float stepsToAngleDeg(float steps) const {
            return (steps / (stepsPerRevolution_ * gearRatio_)) * 360.0f;
        };

        float revolutionToSteps(float revolution) const {
            return (revolution * stepsPerRevolution_ * gearRatio_);
        };

        float stepsToRevolution(float steps) const {
            return (steps / (stepsPerRevolution_ * gearRatio_));
        };

        Generator& getGenerator();
        const Generator& getGenerator() const;

    private:
        Generator& generator_;
        State state_ = State::Undefined;
        uint32_t stepsPerRevolution_ = 200;
        float gearRatio_ = 1.0;
    };
}

#endif // STEPPER_MOTOR_HPP