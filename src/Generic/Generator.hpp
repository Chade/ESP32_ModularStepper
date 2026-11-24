#ifndef STEPPER_GENERATOR_HPP
#define STEPPER_GENERATOR_HPP

#include <FixedPoints.h>
#include <stdint.h>

#include "Driver.hpp"
#include "Helper.hpp"

using UQ20x12 = UFixed<20, 12>;

namespace Stepper {
namespace Generic {

class Generator
{
public:
    Generator() = delete;
    explicit Generator(Driver& driver);
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

    void stop();
    bool pause();
    bool resume();
    bool isPaused() const {
        return state_.currentState == State::Paused;
    }

    void fastStop();
    void fastStop(float deceleration);
    void setFastStopDeceleration(float deceleration);
    float getFastStopDeceleration() const;

    void emergencyStop(bool disableDriver = false);
    bool clearEmergencyStop(bool enableDriver = false);
    bool isEmergencyStopped() const {
        return state_.currentState == State::EmergencyStop;
    }

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

        // Cumulative signed position across both step and velocity modes
        int64_t stepsPosition = 0;

        // Distance mode state
        uint64_t stepsTotal = 0; // total steps requested
        uint64_t stepsDone  = 0; // steps already executed
        uint64_t stepsAcc   = 0; // steps in acceleration phase
        uint64_t stepsConst = 0; // steps in constant velocity phase
        uint64_t stepsDec   = 0; // steps in deceleration phase
    } state_;

    void updateState(GeneratorState& state, State newState) const {
        if (newState != state.currentState) {
            state.previousState = state.currentState;
            state.currentState = newState;
            updateStateEvent(newState);
        }
    };

    GeneratorState getState() const {
        return state_;
    };

    void resetState(bool resetPosition = true) {
        state_.currentState = State::Stopped; // movement state
        state_.previousState = State::Undefined; // movement state

        state_.currentDirection = Direction::Neutral; // current direction
        state_.targetDirection  = Direction::Neutral; // target direction

        state_.doDirectionChange = false; // request direction change

        // Kinematic state
        state_.currentVelocity = 0.0; // steps/s
        state_.targetVelocity  = 0.0; // steps/s
        state_.acceleration    = 0.0; // steps/s^2
        state_.deceleration    = 0.0; // steps/s^2

        // Position state
        if (resetPosition) {
            state_.stepsPosition = 0;
        }

        // Distance mode state
        state_.stepsTotal = 0; // total steps requested
        state_.stepsDone  = 0; // steps already executed
        state_.stepsAcc   = 0; // steps in acceleration phase
        state_.stepsConst = 0; // steps in constant velocity phase
        state_.stepsDec   = 0; // steps in deceleration phase

        pauseRequested_    = false;
        fastStopRequested_ = false;
        hasPausedTask_     = false;
    };

    bool checkState(GeneratorState state) const;

    virtual State waitForState(State state, uint32_t timeout_ms = UINT32_MAX) const = 0;

    float getVelocity() const {
        return static_cast<float>(state_.currentVelocity);
    };

    uint64_t getStepsDone() const {
        return state_.stepsDone;
    };

    int64_t getPositionSteps() const {
        return state_.stepsPosition;
    };

    void setPositionSteps(int64_t steps) {
        state_.stepsPosition = steps;
    };

    Driver& driver() {
        return driver_;
    };

    const Driver& driver() const {
        return driver_;
    };

    void initializeStateBeforeStep(const GeneratorTask& task) {
        initializeStateBeforeStep(task, state_);
    };

    void advanceStateAfterStep(uint32_t steps) {
        advanceStateAfterStep(steps, state_);
    };
    
    void computeStepProfile(GeneratorState& state) const;
    void computeStepProfile_v2(GeneratorState& state) const;
    
private:
    void initializeStateBeforeStep(const GeneratorTask&, GeneratorState& state) const;
    void advanceStateAfterStep(uint32_t steps, GeneratorState& state);
    uint32_t computeStepBatchSize(const GeneratorState& state) const;
    float computeStepPeriodUs(UQ20x12 velocity) const;
    
    UQ20x12 computeDeltaV(UQ20x12 acceleration, uint32_t steps, UQ20x12 velocity) const;
    UQ20x12 computeVelocity(UQ20x12 acceleration, UQ20x12 currentVelocity, UQ20x12 targetVelocity, uint32_t steps) const;
    uint64_t computeRampSteps(UQ20x12 dv, UQ20x12 acceleration) const;

    static void callbackOnStepDone(uint32_t stepsDone, uint32_t& stepsToDo, float& pulsePeriod_us, float& pulsePeriodIncrement_us, void* user_ctx);

    virtual void updateStateEvent(State state) const = 0;

    Driver& driver_;

    GeneratorTask activeTask_;
    GeneratorTask pausedTask_;
    volatile bool pauseRequested_ {false};
    volatile bool fastStopRequested_ {false};
    bool hasPausedTask_ {false};
    UQ20x12 fastStopDeceleration_ {0.0};

    static constexpr UQ20x12 minVelocity_ {1.0};  // steps/s
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_GENERATOR_HPP