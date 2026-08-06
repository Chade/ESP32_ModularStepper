#ifndef STEPPER_GENERATOR_HPP
#define STEPPER_GENERATOR_HPP

#include "StepperDriver_Base.hpp"
#include <stdint.h>
#include <FixedPoints.h>


using UQ20x12 = UFixed<20, 12>;

namespace Stepper
{

    class Generator
    {
    public:
        Generator() = delete;
        Generator(DriverBase& driver);
        virtual ~Generator();

        struct GeneratorTask {
            uint32_t steps = 0;
            UQ20x12 velocity = 0.0;
            UQ20x12 acceleration = 0.0;
            UQ20x12 deceleration = 0.0;
            Direction direction = Direction::Neutral;
        };

        bool run(const GeneratorTask& task);

        // Velocity mode: Ramp to target velocity (steps/s) using given accel/decel (steps/s^2).
        // Call again with targetVelocity = 0 to decelerate to stop.
        bool run(float targetVelocity, float acceleration, float deceleration, Direction direction);

        // Steps mode: Perform 'steps' at up to targetVelocity with accel/decel ramp (steps/s^2).
        // Returns immediately; generator schedules steps until done.
        bool run(uint64_t steps, float targetVelocity, float acceleration, float deceleration, Direction direction);

        struct GeneratorState {
            State currentState = State::Undefined; // movement state
            State previousState = State::Undefined; // movement state

            Direction currentDirection = Direction::Neutral; // current direction
            Direction targetDirection  = Direction::Neutral; // target direction

            bool doDirectionChange = false; // request direction change

            // Kinematic state
            UQ20x12 currentVelocity = 0.0; // steps/s
            UQ20x12 targetVelocity  = 0.0; // steps/s
            UQ20x12 acceleration    = 0.0; // steps/s^2
            UQ20x12 deceleration    = 0.0; // steps/s^2

            // Distance mode state
            uint64_t stepsTotal = 0; // total steps requested
            uint64_t stepsDone  = 0; // steps already executed
            uint64_t stepsAcc   = 0; // steps in acceleration phase
            uint64_t stepsConst = 0; // steps in constant velocity phase
            uint64_t stepsDec   = 0; // steps in deceleration phase

            void updateState(State newState) {
                if (newState != currentState) {
                    previousState = currentState;
                    currentState = newState;    
                }
            };
        } state_;

        GeneratorState getState() const {
            return state_;
        };

        void resetState() {
            state_.currentState = State::Undefined; // movement state
            state_.previousState = State::Undefined; // movement state

            state_.currentDirection = Direction::Neutral; // current direction
            state_.targetDirection  = Direction::Neutral; // target direction

            state_.doDirectionChange = false; // request direction change

            // Kinematic state
            state_.currentVelocity = 0.0; // steps/s
            state_.targetVelocity  = 0.0; // steps/s
            state_.acceleration    = 0.0; // steps/s^2
            state_.deceleration    = 0.0; // steps/s^2

            // Distance mode state
            state_.stepsTotal = 0; // total steps requested
            state_.stepsDone  = 0; // steps already executed
            state_.stepsAcc   = 0; // steps in acceleration phase
            state_.stepsConst = 0; // steps in constant velocity phase
            state_.stepsDec   = 0; // steps in deceleration phase
        };

        bool checkState(GeneratorState state) const;

        float getVelocity() const {
            return static_cast<float>(state_.currentVelocity);
        };

        uint64_t getStepsDone() const {
            return state_.stepsDone;
        };

        DriverBase& getDriver() {
            return driver_;
        };

        const DriverBase& getDriver() const {
            return driver_;
        };

        void initializeStateBeforeStep(const GeneratorTask& task) {
            initializeStateBeforeStep(task, state_);
        };

        void advanceStateAfterStep(uint32_t steps) {
            advanceStateAfterStep(steps, state_);
        };

    private:
        void initializeStateBeforeStep(const GeneratorTask&, GeneratorState& state);
        void advanceStateAfterStep(uint32_t steps, GeneratorState& state);

        UQ20x12 computeStepPeriodUs(UQ20x12 velocity) const;
        static UQ20x12 computeDeltaV(UQ20x12 acceleration, uint32_t steps, UQ20x12 velocity, bool isDeceleration = false);
        static UQ20x12 computeVelocity(UQ20x12 acceleration, UQ20x12 velocity, uint32_t steps, bool isDeceleration);
        static uint64_t computeRampSteps(UQ20x12 dv, UQ20x12 acceleration);

        static void callbackOnStepDone(uint32_t stepsDone, uint32_t& stepsToDo, float& pulsePeriod_us, void* user_ctx);

        DriverBase& driver_;

        static constexpr UQ20x12 minVelocity_ {1.0};  // steps/s
        static constexpr const char* log_tag {"Generator"};
    };
} // namespace Stepper

#endif // STEPPER_GENERATOR_HPP