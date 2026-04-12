#include "StepperGenerator.hpp"
#include "StepperLog.hpp"


namespace Stepper {

    Generator::Generator(DriverBase& driver) : driver_(driver) {
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
            UQ20x12 stepPeriod_us = computeStepPeriodUs(state_.currentVelocity);
            if (stepPeriod_us > 0) {
                driver_.setPulsePeriodUs(static_cast<float>(stepPeriod_us));
                driver_.start(); // resets batch counters, first step triggers callback
                return true;
            }
            else {
                state_.state = State::Stopped;
                driver_.stop();
                return false;
            }
        }
        // Already running — parameters updated, force recalculation on next step
        driver_.forceStepCallback();
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

    // ---- 64-bit safe fixed-point arithmetic helpers ----
    //
    // UQ20x12 uses a 32-bit internal representation (20 integer + 12 fraction bits).
    // At high velocities (200k steps/s) and accelerations (~10k steps/s²),
    // naive fixed-point operations cause:
    //   1) Overflow: dv * dv can reach 4e10, far exceeding UQ20x12 max (~1M)
    //   2) Precision loss: dt = steps/velocity can be <1 LSB (1/4096) and truncate to 0
    //
    // These helpers use 64-bit intermediates to avoid both problems.

    /// Compute dv = acceleration * steps / velocity using 64-bit intermediates.
    /// Avoids the precision loss from computing the tiny dt = steps/velocity first.
    /// Mathematically: dv = acceleration * steps / velocity
    /// In raw representation: dv_raw = acceleration_raw * steps * Scale / velocity_raw
    UQ20x12 Generator::computeDeltaV(UQ20x12 acceleration, uint32_t steps, UQ20x12 velocity) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        uint64_t vel_raw = static_cast<uint64_t>(velocity.getInternal());
        
        if (vel_raw == 0) {
            return UQ20x12::MaxValue;
        }

        uint64_t num = static_cast<uint64_t>(acceleration.getInternal()) * static_cast<uint64_t>(steps);
        uint64_t result_raw = (num * scale) / vel_raw;

        // Clamp to UQ20x12 range to prevent overflow on conversion back to 32-bit
        constexpr uint64_t maxRaw = static_cast<uint64_t>(UQ20x12::MaxValue.getInternal());
        return UQ20x12::fromInternal(static_cast<uint32_t>(result_raw > maxRaw ? maxRaw : result_raw));
    }

    /// Compute s = dv² / (2 * acceleration) as integer step count using 64-bit intermediates.
    /// Avoids overflow from squaring large velocity deltas in 32-bit fixed-point.
    /// Max safe dv: ~4.3 billion raw (full UQ20x12 range), since dv_raw² < 2^64.
    uint64_t Generator::computeRampSteps(UQ20x12 dv, UQ20x12 acceleration) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        uint64_t dv_raw   = static_cast<uint64_t>(dv.getInternal());
        uint64_t acceleration_raw = static_cast<uint64_t>(acceleration.getInternal());

        if (acceleration_raw == 0) {
            return 0;
        }

        // s = dv² / (2*a) = dv_raw² / (2 * Scale * a_raw)
        return (dv_raw * dv_raw) / (2 * scale * acceleration_raw);
    }

    UQ20x12 Generator::computeStepPeriodUs(UQ20x12 velocity) const {
        // Compute period from velocity
        if (velocity == 0.0) {
            return 0.0; // default 0 us when stopped
        }

        // period = 1e6 / v (microseconds per step)
        UQ20x12 period_us = UQ20x12(1'000'000) / velocity;

        // Clamp to a sane range
        UQ20x12 minPeriod_us = driver_.getMinPulsePeriodUs();
        UQ20x12 maxPeriod_us = driver_.getMaxPulsePeriodUs();
        
        period_us = (period_us < minPeriod_us) ? minPeriod_us : period_us;
        period_us = (period_us > maxPeriod_us) ? maxPeriod_us : period_us;

        return period_us;
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
        if (state.currentVelocity == 0.0 && state.targetVelocity > 0.0) {
            state.currentVelocity = minVelocity_; // seed to avoid div by zero
            state.state = State::Accelerating;
            started = true;
        }

        if (state.stepsTotal > 0) {
            state.stepsDone = 0;
            
            // Compute ramp distribution using 64-bit safe helpers (avoids dv*dv overflow)
            UQ20x12 dvAcc = (state.targetVelocity > state.currentVelocity)
                          ? (state.targetVelocity - state.currentVelocity)
                          : (state.currentVelocity - state.targetVelocity);
            uint64_t stepsAcc = (state.acceleration > 0.0) ? computeRampSteps(dvAcc, state.acceleration) : 0;
            uint64_t stepsDec = (state.deceleration > 0.0) ? computeRampSteps(state.targetVelocity, state.deceleration) : 0;

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
                    double ratio = std::min(stepsAcc, stepsDec) / std::max(stepsAcc, stepsDec);
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
        // NOTE: We do NOT compute dt = steps / velocity as an intermediate.
        // At 200k steps/s, dt would be ~5µs = 0.02 LSBs in UQ20x12, truncating to zero.
        // Instead, we compute dv = rate * steps / velocity directly via computeDeltaV(),
        // which uses 64-bit intermediates to preserve precision.

        // Velocity mode
        if (state.stepsTotal == 0) {
            if (state.targetVelocity == 0.0) {
                // Decelerate to stop
                if (state.currentVelocity > 0.0) {
                    state.state = State::Decelerating;

                    UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity);
                    if (dv >= state.currentVelocity) {
                        state.currentVelocity = 0.0;
                    }
                    else {
                        state.currentVelocity = state.currentVelocity - dv;
                    }
                }
                if (state.currentVelocity == 0.0) {
                    state.state = State::Stopped;
                    return false;
                }
            } else {
                if (state.currentDirection != state.targetDirection) {
                    // Need to change direction: decelerate to stand-still first
                    if (state.currentVelocity > 0.0) {
                        state.state = State::Decelerating;

                        UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity);
                        if (dv >= state.currentVelocity) {
                            state.currentVelocity = 0.0;
                        }
                        else {
                            state.currentVelocity = state.currentVelocity - dv;
                        }
                    }
                    if (state.currentVelocity == 0.0) {
                        // Reached stand-still, change direction
                        // We will not transition to State:Stopped, to indicate the movement is not finished yet
                        state.state = State::Running;
                        state.doDirectionChange = true;
                        state.currentVelocity = minVelocity_;
                    }
                }
                else {
                    // Move towards target velocity
                    if (state.currentVelocity < state.targetVelocity) {
                        state.state = State::Accelerating;

                        UQ20x12 dv = computeDeltaV(state.acceleration, steps, state.currentVelocity);
                        state.currentVelocity = state.currentVelocity + dv;
                        if (state.currentVelocity > state.targetVelocity) {
                            state.currentVelocity = state.targetVelocity;
                        }
                    }
                    else if (state.currentVelocity > state.targetVelocity) {
                        state.state = State::Decelerating;

                        UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity);
                        if (dv >= state.currentVelocity) {
                            state.currentVelocity = state.targetVelocity;
                        } else {
                            state.currentVelocity = state.currentVelocity - dv;
                            if (state.currentVelocity < state.targetVelocity) {
                                state.currentVelocity = state.targetVelocity;
                            }
                        }
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
                // Accelerating
                state.state = State::Accelerating;

                UQ20x12 dv = computeDeltaV(state.acceleration, steps, state.currentVelocity);
                state.currentVelocity = state.currentVelocity + dv;
                if (state.currentVelocity > state.targetVelocity) {
                    state.currentVelocity = state.targetVelocity;
                }
            } else if (state.stepsDone <= (state.stepsAcc + state.stepsConst)) {
                // Constant
                state.state = State::Running;

                state.currentVelocity = state.targetVelocity;
            } else if (state.stepsDone <= (state.stepsAcc + state.stepsConst + state.stepsDec)) {
                // Decelerating
                state.state = State::Decelerating;

                UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity);
                if (dv >= state.currentVelocity) {
                    state.currentVelocity = 0.0;
                } else {
                    state.currentVelocity = state.currentVelocity - dv;
                }
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
    }

    float Generator::getVelocity() const {
        return static_cast<float>(state_.currentVelocity);
    }

    uint64_t Generator::getStepsDone() const {
        return state_.stepsDone;
    }

    DriverBase& Generator::getDriver() {
        return driver_;
    }

    const DriverBase& Generator::getDriver() const {
        return driver_;
    }

    uint32_t Generator::callbackOnStepDone(uint32_t stepsNew, float& pulsePeriod_us, void* user_ctx) {
        Generator* self = static_cast<Generator*>(user_ctx);

        // The driver only calls us when the batch threshold is reached.
        // Always recalculate velocity for the accumulated step batch.
        if (self->advanceStateAfterStep(stepsNew, self->state_)) {
            // Check if direction change is pending
            if (self->state_.doDirectionChange) {
                self->state_.currentDirection = self->driver_.changeDirection();
                self->state_.doDirectionChange = false;
            }

            // Compute batch size for next callback.
            // During acc/dec: batch = v/a steps (one velocity quantum).
            // During const:   batch = remaining constant-phase steps (step mode)
            //                        or a large value (velocity mode).
            uint32_t nextBatch = 1;
            if (self->state_.state == State::Accelerating) {
                nextBatch = static_cast<uint32_t>(self->state_.currentVelocity / self->state_.acceleration);
            }
            else if (self->state_.state == State::Decelerating) {
                nextBatch = static_cast<uint32_t>(self->state_.currentVelocity / self->state_.deceleration);
            }
            else if (self->state_.state == State::Running) {
                if (self->state_.stepsTotal > 0) {
                    // Step mode: next batch covers remaining constant-velocity steps
                    uint64_t constEnd = self->state_.stepsAcc + self->state_.stepsConst;
                    uint64_t remaining = (self->state_.stepsDone < constEnd) ? (constEnd - self->state_.stepsDone) : 1;
                    return static_cast<uint32_t>((remaining > UINT32_MAX) ? UINT32_MAX : remaining);
                } else {
                    // Velocity mode: no ramp needed, check infrequently
                    // (forceStepCallback() will override if user calls run() again)
                    return static_cast<uint32_t>((self->state_.currentVelocity + 999) / 1000);
                }
            }

            pulsePeriod_us = static_cast<float>(self->computeStepPeriodUs(self->state_.currentVelocity));
            return nextBatch;
        }
        else {
            self->driver_.stop();
            return 0;
        }
    }


} // namespace Stepper
