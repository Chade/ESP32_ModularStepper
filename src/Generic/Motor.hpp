#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

#include <cassert>
#include <cmath>
#include "Generator.hpp"

namespace Stepper {
namespace Generic {
    class Motor {
    public:
        Motor() = delete;
        explicit Motor(Generic::Generator& generator, uint32_t stepsPerRevolution = 200, float gearRatio = 1.0);
        virtual ~Motor() = default;

        struct MotorTask {
            float revolutions = 0.0;
            float velocity = 0.0;
            float acceleration = 0.0;
            float deceleration = 0.0;
        };

        bool run(const MotorTask& task);
        void run(float targetVelocity, float acceleration, float deceleration);
        void run(float revolutions, float targetVelocity, float acceleration, float deceleration);
        
        void stop() {
            generator_.stop();
        };

        uint32_t getStepsPerRevolution() const {
            return stepsPerRevolution_;
        };
        
        float getGearRatio() const {
            return gearRatio_;
        };

        float getVelocity() const {
            return stepsToRevolutions(generator_.getVelocity());
        };

        float getRevolutions() const {
            return stepsToRevolutions(static_cast<float>(generator_.getPositionSteps()));
        };

        void setRevolutions(float revolutions) {
            generator_.setPositionSteps(static_cast<int64_t>(std::llround(revolutionsToSteps(revolutions))));
        };

        State getState() const {
            return generator_.getState().currentState;
        };

        
        Generic::Generator& generator() {
            return generator_;
        };
        
        const Generic::Generator& generator() const {
            return generator_;
        };
        
    private:
        inline float revolutionsToSteps(float revolutions) const {
            return (revolutions * stepsPerRevolutionGeared_);
        };

        inline float stepsToRevolutions(uint32_t steps) const {
            return (static_cast<float>(steps) / static_cast<float>(stepsPerRevolutionGeared_));
        };

        Generic::Generator& generator_;
        State state_ = State::Undefined;
        uint32_t stepsPerRevolution_;
        float gearRatio_;
        int32_t stepsPerRevolutionGeared_;
    };
} // namespace Generic
} // namespace Stepper

#endif // STEPPER_MOTOR_HPP