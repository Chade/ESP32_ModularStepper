#include "StepperGenerator.h"
#include "StepperLog.h"

namespace Stepper {

    Generator::Generator(DriverInterface& driver) : driver_(driver) {
        esp_log_level_set(log_tag, ESP_LOG_INFO);
        driver_.registerCallbackOnStepDone(callbackOnStepDone, this);
        driver_.init();
    }

    Generator::~Generator() {

    }

    bool Generator::run(const GeneratorTask& task) {
        ESP_LOGI(log_tag, "New task");
        if (initializeStateBeforeStep(task, state_)) {
            // Set driver direction
            driver_.setDirection(state_.targetDirection);
            state_.currentDirection = state_.targetDirection;

            // Calc first step period
            uint32_t stepPeriod_ns = computeStepPeriodNs(state_.currentVelocity);
            if (stepPeriod_ns > 0) {
                // Compute number of steps to reach next velocity level
                if (state_.state == State::Accelerating) {
                    state_.stepsNextUpdate = state_.currentVelocity / state_.acceleration;
                }
                else if (state_.state == State::Decelerating) {
                    state_.stepsNextUpdate = state_.currentVelocity / state_.deceleration;
                }
                driver_.setPulsePeriodNs(stepPeriod_ns);
                driver_.start();
                return true;
            }
            else {
                state_.state = State::Stopped;
                driver_.stop();
                return false;
            }
        }
        return true;
    }

    bool Generator::run(float targetVelocity,
                        float acceleration,
                        float deceleration,
                        Direction direction) {
        GeneratorTask task;
        task.steps = 0;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        task.direction = direction;

        return run(task);
    }

    bool Generator::run(uint64_t steps,
                        float targetVelocity,
                        float acceleration,
                        float deceleration,
                        Direction direction) {
        GeneratorTask task;
        task.steps = steps;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        task.direction = direction;

        return run(task);
    }

    uint32_t Generator::computeStepPeriodNs(float velocity) const {
        // Compute period from velocity
        if (velocity <= 0.0f) {
            return 0; // default 0 ns when stopped
        }
        // period = 1e9 / v (nanoseconds per step)
        uint32_t period_ns = 1e9 / velocity;
        // Clamp to a sane range
        period_ns = std::max(driver_.getMinPulsePeriodNs(), period_ns);
        period_ns = std::min(driver_.getMaxPulsePeriodNs(), period_ns);

        return period_ns;
    }

    bool Generator::initializeStateBeforeStep(const GeneratorTask& task, GeneratorState& state) {
        state.targetVelocity  = task.velocity;
        state.acceleration    = task.acceleration;
        state.deceleration    = task.deceleration;
        state.targetDirection = task.direction;
        state.stepsTotal      = task.steps;
        state.stepsDone       = 0;

        bool started = false;

        // Check if we are starting from stand-still
        if (state.currentVelocity <= 0.0f && state.targetVelocity > 0.0f) {
            state.currentVelocity = minVelocity_; // seed to avoid div by zero
            state.state = State::Accelerating;
            started = true;
        }

        if (state.stepsTotal > 0) {
            state.stepsDone = 0;
            
            // Compute ramp distribution (start from currentVelocity)
            // Steps to accelerate from currentVelocity to targetVelocity
            auto calcStepsNeeded = [](float v0, float v1, float a) -> uint64_t {
                if (a <= 0.0f)
                    return 0;

                float dv = std::max(0.0f, v1 - v0);
                // s = 0.5 * dv^2 / a
                uint64_t s = 0.5 * dv * dv / a;
                return s;
            };
            
            uint64_t stepsAcc = calcStepsNeeded(state.currentVelocity, state.targetVelocity, state.acceleration);
            uint64_t stepsDec = calcStepsNeeded(state.targetVelocity, 0.0f, state.deceleration);

            if (stepsAcc + stepsDec <= state.stepsTotal) {
                state.stepsAcc   = stepsAcc;
                state.stepsDec   = stepsDec;
                state.stepsConst = state.stepsTotal - (stepsAcc + stepsDec);
            } else {
                // Triangular profile: scale acc/dec phases to fit total steps
                if (stepsAcc != 0 && stepsDec == 0) {
                    state.stepsAcc = state.stepsTotal;
                    state.stepsConst = 0;
                    state.stepsDec = 0;
                    state.state = State::Accelerating;
                } else if (stepsAcc == 0 && stepsDec == 0) {
                    state.stepsAcc = 0;
                    state.stepsConst = state.stepsTotal;
                    state.stepsDec = 0;
                    state.state = State::Running;
                } else if (stepsAcc == 0 && stepsDec != 0) {
                    state.stepsAcc = 0;
                    state.stepsConst = 0;
                    state.stepsDec = state.stepsTotal;
                    state.state = State::Decelerating;
                } else {
                    float ratio = static_cast<float>(std::min(stepsAcc, stepsDec)) / static_cast<float>(std::max(stepsAcc, stepsDec));
                    state.stepsAcc = state.stepsTotal * ratio;
                    state.stepsDec = state.stepsTotal - state.stepsAcc;
                    state.stepsConst = 0;
                    state.state = State::Accelerating;
                }
            }
        }
        return started;
    }

    bool Generator::advanceStateAfterStep(uint32_t steps, GeneratorState& state) {
        

        float dt = steps / std::max(state.currentVelocity, minVelocity_);
        // Velocity mode
        if (state.stepsTotal == 0) {
            if (state.targetVelocity <= 0.0f) {
                // Decelerate to stop
                if (state.currentVelocity > 0.0f) {
                    state.state = State::Decelerating;
                    state.currentVelocity = std::max(0.0f, state.currentVelocity - state.deceleration * dt);
                }
                if (state.currentVelocity <= 0.0f) {
                    state.state = State::Stopped;
                    return false;
                }
            } else {
                if (state.currentDirection != state.targetDirection) {
                    // Need to change direction: decelerate to stand-still first
                    if (state.currentVelocity > 0.0f) {
                        state.state = State::Decelerating;
                        state.currentVelocity = std::max(0.0f, state.currentVelocity - state.deceleration * dt);
                    }
                    if (state.currentVelocity <= 0.0f) {
                        // Reached stand-still, change direction
                        state.state = State::Running;
                        state.doDirectionChange = true;
                        state.currentVelocity = minVelocity_;
                    }
                }
                else {
                    // Move towards target velocity
                    if (state.currentVelocity < state.targetVelocity) {
                        state.state = State::Accelerating;
                        state.currentVelocity = std::min(state.targetVelocity, state.currentVelocity + state.acceleration * dt);
                    }
                    else if (state.currentVelocity > state.targetVelocity) {
                        state.state = State::Decelerating;
                        state.currentVelocity = std::max(state.targetVelocity, state.currentVelocity - state.deceleration * dt);
                    }
                    else {
                        state.state = State::Running;
                    }
                }
            }
        }
        // Step mode
        else {
            state.stepsDone += steps;
            // Determine phase by stepsDone
            if (state.stepsDone <= state.stepsAcc) {
                // Accelerating: v^2 = v0^2 + 2 a s; step-by-step approx
                state.state = State::Accelerating;
                state.currentVelocity = std::min(state.targetVelocity, state.currentVelocity + state.acceleration * dt);
            } else if (state.stepsDone <= (state.stepsAcc + state.stepsConst)) {
                // Constant
                state.state = State::Running;
                state.currentVelocity = state.targetVelocity;
            } else if (state.stepsDone <= (state.stepsAcc + state.stepsConst + state.stepsDec)) {
                // Decelerating
                state.state = State::Decelerating;
                state.currentVelocity = std::max(0.0f, state.currentVelocity - state.deceleration * dt);
            }

            if (state.stepsDone >= state.stepsTotal) {
                state.state = State::Stopped;
                return false;
            }
        }
        return true;
    }

    State Generator::getState() const {
        return state_.state;
    }

    void Generator::resetState() {
        state_.state = State::Undefined; // movement state

        state_.currentDirection = Direction::Neutral; // current direction
        state_.targetDirection  = Direction::Neutral; // target direction

        state_.doDirectionChange = false; // request direction change

        // Kinematic state
        state_.currentVelocity = 0; // steps/s
        state_.targetVelocity  = 0; // steps/s
        state_.acceleration    = 0; // steps/s^2
        state_.deceleration    = 0; // steps/s^2

        // Distance mode state
        state_.stepsTotal = 0; // total steps requested
        state_.stepsDone  = 0; // steps already executed
        state_.stepsAcc   = 0; // steps in acceleration phase
        state_.stepsConst = 0; // steps in constant velocity phase
        state_.stepsDec   = 0; // steps in deceleration phase

        state_.stepsCurrent = 0;
        state_.stepsNextUpdate = 0;
    }

    float Generator::getVelocity() const {
        return state_.currentVelocity;
    }

    uint32_t Generator::callbackOnStepDone(uint32_t stepsNew, uint32_t pulsePeriod_ns, void* user_ctx) {
        Generator* self = static_cast<Generator*>(user_ctx);
        
        // Preserve steps for next cycle
        self->state_.stepsCurrent += stepsNew;

        if ((self->state_.stepsCurrent >= self->state_.stepsNextUpdate) && self->state_.state != State::Running) {

            // Propagate internal states for next period
            if (self->advanceStateAfterStep(self->state_.stepsCurrent, self->state_)) {
                // Check if direction change is pending
                if (self->state_.doDirectionChange) {
                    self->state_.currentDirection = self->driver_.changeDirection();
                    self->state_.doDirectionChange = false;
                }

                // Compute number of steps to reach next velocity level
                if (self->state_.state == State::Accelerating) {
                    self->state_.stepsNextUpdate = self->state_.currentVelocity / self->state_.acceleration;
                }
                else if (self->state_.state == State::Decelerating) {
                    self->state_.stepsNextUpdate = self->state_.currentVelocity / self->state_.deceleration;
                }
                else {
                    self->state_.stepsNextUpdate = self->state_.stepsConst;
                }

                // Reset step counter
                self->state_.stepsCurrent = 0;

                // Compute and return next step period
                return self->computeStepPeriodNs(self->state_.currentVelocity);
            }
            else {
                self->driver_.stop();
                return 0;
            }
        }
        return pulsePeriod_ns;
    }


} // namespace Stepper
