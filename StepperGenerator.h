#ifndef STEPPER_GENERATOR_H
#define STEPPER_GENERATOR_H

#include "StepperDriver_Interface.h"
#include <stdint.h>

namespace Stepper
{

    class Generator
    {
    public:
        Generator() = delete;
        Generator(DriverInterface& driver);
        ~Generator();

        struct GeneratorTask {
            uint64_t steps = 0;
            float velocity = 0;
            float acceleration = 0;
            float deceleration = 0;
            Direction direction = Direction::Neutral;
        };

        bool run(const GeneratorTask& task);

        // Velocity mode: Ramp to target velocity (steps/s) using given accel/decel (steps/s^2).
        // Call again with targetVelocity = 0 to decelerate to stop.
        bool run(float targetVelocity, float acceleration, float deceleration, Direction direction);

        // Steps mode: Perform 'steps' at up to targetVelocity with accel/decel ramp (steps/s^2).
        // Returns immediately; generator schedules steps until done.
        bool run(uint64_t steps, float targetVelocity, float acceleration, float deceleration, Direction direction);

        State getState() const;
        void resetState();

        float getVelocity() const;

    private:

        struct GeneratorState {
            State state = State::Undefined; // movement state

            Direction currentDirection = Direction::Neutral; // current direction
            Direction targetDirection  = Direction::Neutral; // target direction

            bool doDirectionChange = false; // request direction change

            // Kinematic state
            float currentVelocity = 0; // steps/s
            float targetVelocity  = 0; // steps/s
            float acceleration    = 0; // steps/s^2
            float deceleration    = 0; // steps/s^2

            // Distance mode state
            uint64_t stepsTotal = 0; // total steps requested
            uint64_t stepsDone  = 0; // steps already executed
            uint64_t stepsAcc   = 0; // steps in acceleration phase
            uint64_t stepsConst = 0; // steps in constant velocity phase
            uint64_t stepsDec   = 0; // steps in deceleration phase

            uint32_t stepsCurrent = 0;
            uint32_t stepsNextUpdate = 0;
        } state_;

        uint32_t computeStepPeriodNs(float velocity) const;
        bool initializeStateBeforeStep(const GeneratorTask&, GeneratorState& state);
        bool advanceStateAfterStep(uint32_t steps, GeneratorState& state);

        static uint32_t callbackOnStepDone(uint32_t stepsNew, uint32_t pulsePeriod_ns, void* user_ctx);

        DriverInterface& driver_;

        static constexpr float minVelocity_ = 1.0f;  // steps/s
        static constexpr const char* log_tag = "Generator";
    };
} // namespace Stepper

#endif // STEPPER_GENERATOR_H