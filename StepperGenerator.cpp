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
        ESP_LOGI(log_tag, "Initialize generator state");
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
    UQ20x12 Generator::computeDeltaV(UQ20x12 rate, uint32_t steps, UQ20x12 velocity, bool isDeceleration) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        
        uint64_t acc_raw = static_cast<uint64_t>(rate.getInternal());
        if (acc_raw == 0) {
            return UQ20x12(0);
        }
        
        uint64_t vel_raw = static_cast<uint64_t>(velocity.getInternal());
        if (vel_raw == 0) {
            // Standstill acceleration: v_new = sqrt(2 * a * s)
            // In raw fixed-point: dv_raw = sqrt(2 * acc_raw * steps * scale)
            uint64_t arg = 2 * acc_raw * static_cast<uint64_t>(steps) * scale;
            uint64_t result_raw = static_cast<uint64_t>(std::sqrt(arg));
            constexpr uint64_t max_raw = static_cast<uint64_t>(UQ20x12::MaxValue.getInternal());
            return UQ20x12::fromInternal(static_cast<uint32_t>(result_raw > max_raw ? max_raw : result_raw));
        }
            
        uint64_t result_raw = (acc_raw * static_cast<uint64_t>(steps) * scale) / vel_raw;
        
        // Clamp to UQ20x12 range to prevent overflow on conversion back to 32-bit
        constexpr uint64_t max_raw = static_cast<uint64_t>(UQ20x12::MaxValue.getInternal());
        return UQ20x12::fromInternal(static_cast<uint32_t>(result_raw > max_raw ? max_raw : result_raw));
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

   /// Compute exact velocity change dv over 'steps' using 64-bit fixed-point kinematics.
   /// Acceleration: dv = sqrt(v² + 2*a*s) - v
   /// Deceleration: dv = v - sqrt(max(0, v² - 2*d*s))

    /*UQ20x12 Generator::computeDeltaV(UQ20x12 rate, uint32_t steps, UQ20x12 velocity, bool isDeceleration) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        
        uint64_t rate_raw = static_cast<uint64_t>(rate.getInternal());
        if (rate_raw == 0 || steps == 0) {
            return UQ20x12(0);
        }

        uint64_t vel_raw = static_cast<uint64_t>(velocity.getInternal());
        uint64_t two_a_s_scale = 2 * rate_raw * static_cast<uint64_t>(steps) * scale;

        uint64_t dv_raw = 0;

        if (!isDeceleration) {
            // Acceleration: v_new = sqrt(v² + 2*a*s)
            uint64_t v_sq = vel_raw * vel_raw;
            uint64_t v_new_sq = 0;
            if (UINT64_MAX - v_sq < two_a_s_scale) {
                v_new_sq = UINT64_MAX;
            } else {
                v_new_sq = v_sq + two_a_s_scale;
            }
            uint64_t v_new_raw = static_cast<uint64_t>(std::sqrt(v_new_sq));
            dv_raw = (v_new_raw > vel_raw) ? (v_new_raw - vel_raw) : 0;
        } else {
            // Deceleration: v_new = sqrt(max(0, v² - 2*d*s))
            uint64_t v_sq = vel_raw * vel_raw;
            if (v_sq <= two_a_s_scale) {
                return velocity; // Decelerates all the way to 0
            }
            uint64_t v_new_sq = v_sq - two_a_s_scale;
            uint64_t v_new_raw = static_cast<uint64_t>(std::sqrt(v_new_sq));
            dv_raw = (vel_raw > v_new_raw) ? (vel_raw - v_new_raw) : 0;
        }

        // Clamp to UQ20x12 range to prevent overflow on conversion back to 32-bit
        constexpr uint64_t max_raw = static_cast<uint64_t>(UQ20x12::MaxValue.getInternal());
        return UQ20x12::fromInternal(static_cast<uint32_t>(dv_raw > max_raw ? max_raw : dv_raw));
    }*/

    /// Compute s = dv² / (2 * acceleration) as integer step count using 64-bit intermediates.
    /// Avoids overflow from squaring large velocity deltas in 32-bit fixed-point.
    /// Max safe dv: ~4.3 billion raw (full UQ20x12 range), since dv_raw² < 2^64.
    uint64_t Generator::computeRampSteps(UQ20x12 dv, UQ20x12 acceleration) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        uint64_t dv_raw   = static_cast<uint64_t>(dv.getInternal());
        
        uint64_t acc_raw = static_cast<uint64_t>(acceleration.getInternal());
        if (acc_raw == 0) {
            return 0;
        }

        // s = dv² / (2*a) = dv_raw² / (2 * Scale * a_raw)
        return (dv_raw * dv_raw) / (2 * scale * acc_raw);
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

        bool Generator::checkState(GeneratorState state) const {
        bool ret = true;
        if (state.targetVelocity < 0.0f) {
            ESP_LOGW(log_tag, "Generator state invalid: targetVelocity needs to be positive");
            ret = false;
        }
        else if (state.targetVelocity == 0.0f) {
            if (state.stepsTotal > 0) {
                ESP_LOGW(log_tag, "Generator state invalid: stepsTotal > 0 but targetVelocity == 0");
                ret = false;
            }
        }
        else if (state.targetVelocity > 0.0f ) {
            if (state.targetDirection == Direction::Neutral) {
                ESP_LOGW(log_tag, "Generator state invalid: targetVelocity > 0 but targetDirection == Neutral");
                ret = false;
            }
            if (!driver_.checkPulsePeriod(static_cast<float>(computeStepPeriodUs(state.targetVelocity)))) {
                ESP_LOGW(log_tag, "Generator state invalid: targetVelocity results in invalid pulse period");
                ret = false;
            }
        }
        return ret;
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

                    UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity, true);
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

                        UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity, true);
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

                        UQ20x12 dv = computeDeltaV(state.acceleration, steps, state.currentVelocity, false);
                        state.currentVelocity = state.currentVelocity + dv;
                        if (state.currentVelocity > state.targetVelocity) {
                            state.currentVelocity = state.targetVelocity;
                        }
                    }
                    else if (state.currentVelocity > state.targetVelocity) {
                        state.state = State::Decelerating;

                        UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity, true);
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

                UQ20x12 dv = computeDeltaV(state.acceleration, steps, state.currentVelocity, false);
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

                UQ20x12 dv = computeDeltaV(state.deceleration, steps, state.currentVelocity, true);
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

    void Generator::callbackOnStepDone(uint32_t stepsDone, uint32_t& stepsToDo, float& pulsePeriod_us, void* user_ctx) {
        Generator* self = static_cast<Generator*>(user_ctx);

        // The driver only calls us when the batch threshold is reached.
        // Always recalculate velocity for the accumulated step batch.
        self->advanceStateAfterStep(stepsDone, self->state_);
        
        // Check if direction change is pending
        if (self->state_.doDirectionChange) {
            self->state_.currentDirection = self->driver_.changeDirection();
            self->state_.doDirectionChange = false;
        }
        
        // Compute next pulse period based on current velocity
        pulsePeriod_us = static_cast<float>(self->computeStepPeriodUs(self->state_.currentVelocity));

        // Compute batch size for next callback.
        // During acc/dec: batch = v/a steps (one velocity quantum).
        // During const:   batch = remaining constant-phase steps (step mode)
        //                         or a large value (velocity mode).
        if (self->state_.state == State::Accelerating) {
            UQ20x12 steps = (self->state_.currentVelocity + self->state_.acceleration - 1.0f) / self->state_.acceleration;
            stepsToDo = steps.getInteger();
        }
        else if (self->state_.state == State::Decelerating) {
            UQ20x12 steps = (self->state_.currentVelocity + self->state_.deceleration - 1.0f) / self->state_.deceleration;
            stepsToDo = steps.getInteger();
        }
        else if (self->state_.state == State::Running) {
            if (self->state_.stepsTotal > 0) {
                // Step mode: next batch covers remaining constant-velocity steps
                uint64_t constEnd = self->state_.stepsAcc + self->state_.stepsConst;
                uint64_t remaining = (self->state_.stepsDone < constEnd) ? (constEnd - self->state_.stepsDone) : 1;
                stepsToDo = static_cast<uint32_t>((remaining > UINT32_MAX) ? UINT32_MAX : remaining);
            } else {
                // Velocity mode: no ramp needed, check infrequently
                // (forceStepCallback() will override if user calls run() again)
                stepsToDo = static_cast<uint32_t>((self->state_.currentVelocity + 999) / 1000);
            }
        }
        else if (self->state_.state == State::Stopped) {
            stepsToDo = 0;
        }
        else {
            ESP_LOGW(log_tag, "Unexpected generator state: %d", static_cast<int>(self->state_.state));
        }
    }


} // namespace Stepper
