#include "Generator.hpp"
#include "Helper.hpp"
#include "Log.hpp"


namespace Stepper {
namespace Generic {

    Generator::Generator(Driver& driver) : driver_(driver) {
        driver_.registerCallbackOnStepDone(callbackOnStepDone, this);
        driver_.init();
    }

    Generator::~Generator() {

    }

    bool Generator::run(const GeneratorTask& task) {
        LOG_INFO("Initialize generator state");
        // Initialize internal state from task
        initializeStateBeforeStep(task, state_);

        // Perform some sanity checks
        checkState(state_);

        // Check if we are starting from stand-still
        if (state_.previousState == State::Stopped) {
            LOG_INFO("Starting driver");
            
            // Calc first step period
            driver_.reset(); // resets batch counters, first step triggers callback
            driver_.setPulsePeriodUs(computeStepPeriodUs(state_.currentVelocity));
            driver_.setDirection(state_.targetDirection);
            driver_.start();
        }
        else if (state_.currentState == State::Stopped) {
            LOG_INFO("Stopping driver");
            driver_.stop();
            return false;
        }
        else {
            LOG_INFO("Updating driver");
            driver_.forceStepCallback();
        }
        return true;
    }

    bool Generator::run(float targetVelocity,
                        float acceleration,
                        float deceleration,
                        Direction direction) {
        if (targetVelocity < 0.0f || acceleration < 0.0f || deceleration < 0.0f) {
            return false;
        }

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
        if (targetVelocity < 0.0f || acceleration < 0.0f || deceleration < 0.0f) {
            return false;
        }

        GeneratorTask task;
        task.steps = steps;
        task.velocity = targetVelocity;
        task.acceleration = acceleration;
        task.deceleration = deceleration;
        task.direction = direction;

        return run(task);
    }

    void Generator::stop() {
        driver_.stop();
        state_.currentState = State::Stopped;
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

        uint64_t acc_raw = static_cast<uint64_t>(acceleration.getInternal());
        if (acc_raw == 0) {
            return UQ20x12::fromInternal(0);
        }
        
        uint64_t vel_raw = static_cast<uint64_t>(velocity.getInternal());
        if (vel_raw == 0) {
            return UQ20x12::MaxValue;
        }
        
        uint64_t result_raw;
        if (vel_raw < acc_raw) {
            // Standstill acceleration: v_new = sqrt(2 * a * s)
            // In raw fixed-point: dv_raw = sqrt(2 * acc_raw * steps * scale)
            uint64_t arg = 2 * acc_raw * static_cast<uint64_t>(steps) * scale;
            //uint64_t result_raw = static_cast<uint64_t>(std::sqrt(arg));
            result_raw = isqrt64(arg);
            result_raw = (result_raw > vel_raw) ? (result_raw - vel_raw) : (vel_raw - result_raw);
        }
        else {
            result_raw = (acc_raw * static_cast<uint64_t>(steps) * scale) / vel_raw;
        }
        
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

    UQ20x12 Generator::computeVelocity(UQ20x12 acceleration, UQ20x12 currentVelocity, UQ20x12 targetVelocity, uint32_t steps) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        
        if (steps == 0) {
            return currentVelocity;
        }

        if (acceleration == 0) {
            return targetVelocity;
        }
        
        uint64_t acc_raw = static_cast<uint64_t>(acceleration.getInternal());
        uint64_t vel_cur_raw = static_cast<uint64_t>(currentVelocity.getInternal());
        uint64_t vel_tar_raw = static_cast<uint64_t>(targetVelocity.getInternal());
        uint64_t two_a_s_raw = 2 * acc_raw * steps * scale;

        uint64_t v_cur_sq_raw = vel_cur_raw * vel_cur_raw;

        uint64_t v_new_raw = 0;
        if (currentVelocity < targetVelocity) {
            // Acceleration: v_new = sqrt(v² + 2*a*s)
            if (UINT64_MAX - v_cur_sq_raw < two_a_s_raw) {
                v_new_raw = isqrt64(UINT64_MAX);
            }
            else {
                v_new_raw = isqrt64(v_cur_sq_raw + two_a_s_raw);
            }
            v_new_raw = std::min(v_new_raw, vel_tar_raw);
        } else if (currentVelocity > targetVelocity) {
            // Deceleration: v_new = sqrt(max(0, v² - 2*d*s))
            if (v_cur_sq_raw <= two_a_s_raw) {
               v_new_raw = 0;
            }
            else {
                v_new_raw = isqrt64(v_cur_sq_raw - two_a_s_raw);
            }
            v_new_raw = std::max(v_new_raw, vel_tar_raw);
        }
        else {
            return currentVelocity;
        }

        // Clamp to UQ20x12 range to prevent overflow on conversion back to 32-bit
        constexpr uint64_t max_raw = static_cast<uint64_t>(UQ20x12::MaxValue.getInternal());
        return UQ20x12::fromInternal(static_cast<uint32_t>(v_new_raw > max_raw ? max_raw : v_new_raw));
    }

    /// Compute s = dv² / (2 * acceleration) as integer step count using 64-bit intermediates.
    /// Avoids overflow from squaring large velocity deltas in 32-bit fixed-point.
    /// Max safe dv: ~4.3 billion raw (full UQ20x12 range), since dv_raw² < 2^64.
    uint64_t Generator::computeRampSteps(UQ20x12 dv, UQ20x12 acceleration) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        
        if (acceleration == 0) {
            return 0;
        }

        uint64_t dv_raw   = static_cast<uint64_t>(dv.getInternal());
        uint64_t acc_raw = static_cast<uint64_t>(acceleration.getInternal());

        // s = dv² / (2*a) = dv_raw² / (2 * a_raw * scale)
        return (dv_raw * dv_raw) / (2 * acc_raw * scale);
    }

    float Generator::computeStepPeriodUs(UQ20x12 velocity) const {
        // Compute period from velocity
        if (velocity == 0) {
            return 0.0; // default 0 us when stopped
        }

        // period = 1e6 / v (microseconds per step)
        UQ20x12 period_us = UQ20x12(1'000'000) / velocity;

        // Clamp to a sane range
        UQ20x12 minPeriod_us = driver_.getMinPulsePeriodUs();
        UQ20x12 maxPeriod_us = driver_.getMaxPulsePeriodUs();
        
        period_us = (period_us < minPeriod_us) ? minPeriod_us : period_us;
        period_us = (period_us > maxPeriod_us) ? maxPeriod_us : period_us;

        return static_cast<float>(period_us);
    }

    void Generator::computeStepProfile(GeneratorState& state) {
        uint64_t stepsAcc = 0;
        // Check if we need to change direction
        if (state.targetDirection != state.currentDirection) {
            LOG_INFO("Direction change required");
            // We will need to decelerate first
            state.currentState = State::Decelerating;
            // Calc steps needed to accelerate back to target velocity after changing direction
            stepsAcc = computeRampSteps(state.targetVelocity, state.acceleration);
        }
        else {
            // Calc steps needed to accelerate to target velocity
            if (state.targetVelocity == 0 && state.currentVelocity == 0) {
                state.currentState = State::Stopped;
                // This should not happen, as we should not be running in step mode with zero velocity, but handle it gracefully
            }
            else if (state.targetVelocity > state.currentVelocity) {
                state.currentState = State::Accelerating;
                // Calc steps needed to accelerate to target velocity
                UQ20x12 dv = state.targetVelocity - state.currentVelocity;
                stepsAcc = computeRampSteps(dv, state.acceleration);
            }
            else if (state.targetVelocity == state.currentVelocity) {
                state.currentState = State::Running;
                // No acceleration needed
                stepsAcc = 0;
            }
            else if (state.targetVelocity < state.currentVelocity) {
                state.currentState = State::Decelerating;
                // Calc steps needed to decelerate to target velocity
                UQ20x12 dv = state.currentVelocity - state.targetVelocity;
                stepsAcc = computeRampSteps(dv, state.deceleration);
            }
        }

        // Calc steps needed to decelerate to stand-still at the end of the movement
        uint64_t stepsDec = computeRampSteps(state.targetVelocity, state.deceleration);

        if (stepsAcc + stepsDec <= state.stepsTotal) {
            LOG_INFO("Calc trapezoid profile");
            // Trapezoid profile: full acc + dec + const velocity
            state.stepsAcc   = stepsAcc;
            state.stepsDec   = stepsDec;
            state.stepsConst = state.stepsTotal - (stepsAcc + stepsDec);
        } else {
            // Triangular profile: scale acc/dec phases to fit total steps
            LOG_INFO("Calc triangular profile");
            if (stepsAcc > 0 && stepsDec == 0) {
                state.stepsAcc = state.stepsTotal;
                state.stepsDec = 0;
                state.stepsConst = 0;
            } else if (stepsAcc == 0 && stepsDec == 0) {
                state.stepsAcc = 0;
                state.stepsDec = 0;
                state.stepsConst = state.stepsTotal;
            } else if (stepsAcc == 0 && stepsDec > 0) {
                state.stepsAcc = 0;
                state.stepsDec = state.stepsTotal;
                state.stepsConst = 0;
            } else {
                // First prio for decelerating to standstill after movement
                if (state.stepsTotal >= stepsDec) {
                    state.stepsAcc = state.stepsTotal - stepsDec;
                    state.stepsDec = stepsDec;
                    state.stepsConst = 0;
                }
                float ratioAcc = static_cast<float>(stepsAcc) / static_cast<float>(stepsAcc + stepsDec);
                float ratioDec = static_cast<float>(stepsDec) / static_cast<float>(stepsAcc + stepsDec);

                state.stepsAcc = state.stepsTotal * ratioAcc;
                state.stepsDec = state.stepsTotal * ratioDec;
                state.stepsConst = state.stepsTotal - (state.stepsAcc + state.stepsDec);
            }
            LOG_INFO("Velocity profile: [%llu|%llu|%llu]", state.stepsAcc, state.stepsConst, state.stepsDec);
            }
    }

    void Generator::computeStepProfile_v2(GeneratorState& state) {
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096
        
        uint64_t v_start_raw = static_cast<uint64_t>((state.currentDirection == state.targetDirection) ? state.currentVelocity.getInternal() : 0);
        uint64_t v_max_raw = static_cast<uint64_t>(state.targetVelocity.getInternal());
        uint64_t acc_raw = static_cast<uint64_t>(state.acceleration.getInternal());
        uint64_t dec_raw = static_cast<uint64_t>(state.deceleration.getInternal());
        uint64_t steps_total_raw = state.stepsTotal * scale;
        
        uint64_t v_start_sq_raw = v_start_raw * v_start_raw;
        uint64_t v_max_sq_raw = v_max_raw * v_max_raw;
        uint64_t v_end_sq_raw = 2 * dec_raw * steps_total_raw;
        
        if (v_end_sq_raw < v_start_sq_raw) {
            // Even with full deceleration the velocity will not be reduced to zero 
            UQ20x12 v_remaining = UQ20x12::fromInternal(isqrt64(v_start_sq_raw - v_end_sq_raw));
            LOG_WARN("Steps not sufficient to decelerate to standstill");
            LOG_WARN("Remaining velocity: %.2f steps/s", static_cast<float>(v_remaining));
            state.stepsAcc = 0;
            state.stepsDec = state.stepsTotal;
            state.stepsConst = 0;
            return;
        }
        
        uint64_t steps_peak_raw = 0;
        uint64_t v_peak_sq_raw = 0;
        if (acc_raw == 0 && dec_raw == 0) {
            // No acceleration and deceleration phase, only const velocity
            state.stepsAcc = 0;
            state.stepsDec = 0;
            state.stepsConst = state.stepsTotal;
            return;
        }
        else if (acc_raw == 0) {
            steps_peak_raw = 0;
            v_peak_sq_raw = 2 * dec_raw * steps_total_raw;
        }
        else if (dec_raw == 0) {
            steps_peak_raw = steps_total_raw;
            v_peak_sq_raw = v_start_sq_raw + 2 * acc_raw * steps_total_raw;
        }
        else {
            steps_peak_raw = (v_end_sq_raw - v_start_sq_raw) / (2 * (acc_raw + dec_raw));
            v_peak_sq_raw = 2 * dec_raw * (steps_total_raw - steps_peak_raw);
        }

        if (v_peak_sq_raw <= v_max_sq_raw) {
            LOG_INFO("Calc triangular profile");
            state.stepsAcc = steps_peak_raw / scale;
            state.stepsDec = state.stepsTotal - state.stepsAcc;
            state.stepsConst = 0;
            state.targetVelocity = isqrt64(v_peak_sq_raw) / scale;
            LOG_INFO("Target velocity will not be reached, new target velocity: %.2f steps/s", static_cast<float>(state.targetVelocity));
        } else {
            LOG_INFO("Calc trapezoid profile");
            if (v_start_raw < v_max_raw) {
                // (v_max - v_start)² / (2*acc_raw) = (v_max² -2*vmax*v_start + v_start²) / (2*acc_raw)
                state.stepsAcc = (acc_raw > 0) ? (v_max_sq_raw - 2 * v_max_raw * v_start_raw + v_start_sq_raw) / (2 * acc_raw * scale) : 0;
            }
            else {
                // (v_start - v_max)² / (2*dec_raw) = (v_max² -2*vmax*v_start + v_start²) / (2*dec_raw)
                state.stepsAcc = (dec_raw > 0) ? (v_start_sq_raw - 2 * v_max_raw * v_start_raw + v_max_sq_raw) / (2 * dec_raw * scale) : 0;
            }
            state.stepsDec = (dec_raw > 0) ? v_max_sq_raw / (2 * dec_raw * scale) : 0;
            state.stepsConst = state.stepsTotal - state.stepsAcc - state.stepsDec;
        }
    }

    bool Generator::checkState(GeneratorState state) const {
        bool ret = true;
        if (state.targetVelocity == 0) {
            if (state.currentVelocity > state.targetVelocity && state.deceleration == 0) {
                LOG_WARN("Generator state invalid: Deceleration == 0, will result in abrupt velocity change");
                ret = false;
            }
            if (state.stepsTotal > 0) {
                LOG_WARN("Generator state invalid: stepsTotal > 0 but targetVelocity == 0");
                ret = false;
            }
        }
        else if (state.targetVelocity > 0 ) {
            if (state.currentVelocity < state.targetVelocity  && state.acceleration == 0) {
                LOG_WARN("Generator state invalid: Acceleration == 0, will result in abrupt velocity change");
                ret = false;
            }
            if (state.targetDirection == Direction::Neutral) {
                LOG_WARN("Generator state invalid: targetVelocity > 0 but targetDirection == Neutral");
                ret = false;
            }
            if (!driver_.checkPulsePeriod(computeStepPeriodUs(state.targetVelocity))) {
                LOG_WARN("Generator state invalid: targetVelocity results in invalid pulse period");
                ret = false;
            }
        }

        return ret;
    }

    void Generator::initializeStateBeforeStep(const GeneratorTask& task, GeneratorState& state) const {
        state.targetVelocity  = task.velocity;
        state.acceleration    = task.acceleration;
        state.deceleration    = task.deceleration;
        state.targetDirection = task.direction;
        state.stepsTotal      = task.steps;

        // Check if we are starting from stand-still
        if (state.currentVelocity == 0) {
            LOG_INFO("Starting from stand-still");
            state.currentDirection = state.targetDirection;
            state.previousState = State::Stopped;
        }
        else {
            LOG_INFO("Current velocity: %f", static_cast<float>(state.currentVelocity));
            state.previousState = state.currentState;
        }

        // Check if running in step or constant velocity mode
        if (state.stepsTotal > 0) {
            LOG_INFO("Running in step mode");
            state.stepsDone = 0;
            computeStepProfile_v2(state);
        }
        else {
            LOG_INFO("Running in constant velocity mode");
        }

        // Set state
        if (state.targetDirection != state.currentDirection) {
            LOG_INFO("Direction change required");
            state.currentState = State::Decelerating;
        }
        else if (state.currentVelocity == 0 && state.targetVelocity == 0) {
            state.currentState = State::Stopped;
        }
        else if (state.currentVelocity == 0 && state.targetVelocity > 0) {
            state.currentState = State::Accelerating;
            state.currentVelocity = computeVelocity(state.acceleration, state.currentVelocity, state.targetVelocity, 1); // Preseed velocity if starting from stand-still
        }
        else if (state.currentVelocity < state.targetVelocity) {
            state.currentState = State::Accelerating;
        }
        else if (state.currentVelocity > state.targetVelocity) {
            state.currentState = State::Decelerating;
        }
        else {
            state.currentState = State::Running;
        }

    }

    void Generator::advanceStateAfterStep(uint32_t steps, GeneratorState& state) const {
        UQ20x12 lastVelocity = state.currentVelocity;

        if (state.currentDirection == Direction::Neutral || state.targetDirection == Direction::Neutral) {
            LOG_INFO("Driver switched into neutral (disabled)");
            state.targetVelocity = 0;
            state.currentVelocity = 0;
        }

        if (state.stepsTotal > 0) {
            if (state.currentDirection == state.targetDirection) {
                state.stepsDone += steps;
                // Determine phase by stepsDone
                if (state.stepsDone < state.stepsAcc) {
                    // Accelerating
                }
                else if (state.stepsDone <= (state.stepsAcc + state.stepsConst)) {
                    // Constant
                    state.currentVelocity = state.targetVelocity;
                } else if (state.stepsDone < (state.stepsAcc + state.stepsConst + state.stepsDec)) {
                    // Decelerating
                    state.targetVelocity = 0;
                } else if (state.stepsDone >= state.stepsTotal) {
                    // Stopping
                    LOG_INFO("Target steps reached [%llu/%llu/%lu]", state.stepsDone, state.stepsTotal, steps);
                    state.targetVelocity = 0;
                    state.currentVelocity = 0;
                }
            }
        }

        if (state.currentDirection != state.targetDirection) {
            // Need to change direction: decelerate to stand-still first
            if (state.currentVelocity > 0) {
                state.updateState(State::Decelerating);
                state.currentVelocity = computeVelocity(state.deceleration, state.currentVelocity, state.targetVelocity, steps);
            }
            if (state.currentVelocity == 0) {
                LOG_INFO("Do direction change");
                // Reached stand-still, change direction
                // We will not transition to State:Stopped, to indicate the movement is not finished yet
                state.updateState(State::Accelerating);
                state.doDirectionChange = true;
                state.currentVelocity = minVelocity_;
            }
        }
        else {
            // Accelerate towards target velocity
            if (state.currentVelocity < state.targetVelocity) {
                state.updateState(State::Accelerating);
                state.currentVelocity = computeVelocity(state.acceleration, state.currentVelocity, state.targetVelocity, steps);
            }
            // Decelerate towards target velocity
            else if (state.currentVelocity > state.targetVelocity) {
                state.updateState(State::Decelerating);
                state.currentVelocity = computeVelocity(state.deceleration, state.currentVelocity, state.targetVelocity, steps);
            }
            // System reached target velocity
            else if (state.currentVelocity == state.targetVelocity) {
                state.updateState(State::Running);
            }

            // Check if system has reached standstill
            if (state.currentVelocity == 0 && state.targetVelocity == 0) {
                if (state.stepsTotal > 0 && state.stepsDone < state.stepsTotal) {
                    LOG_INFO("Current velocity: 0 steps/s but still %llu steps to go", (state.stepsTotal - state.stepsDone));
                    state.currentVelocity = lastVelocity;
                }
                else {
                    LOG_INFO("Standstill reached (%llu/%llu/%lu)", state.stepsDone, state.stepsTotal, steps);
                    state.updateState(State::Stopped);
                }
            }
        }
    }

   uint32_t Generator::computeStepBatchSize(const GeneratorState& state) const {
        // Compute batch size for next callback.
        // During acc/dec: batch = v/a steps (one velocity quantum).
        // During const:   batch = remaining constant-phase steps (step mode)
        //                         or a large value (velocity mode).
        uint32_t stepsToDo = 0;
        
        if (state.currentState == State::Accelerating) {
            if (state.acceleration == 0) {
                return 1;
            }

            UQ20x12 steps = (state.currentVelocity + state.acceleration ) / state.acceleration;
            stepsToDo = steps.getInteger();
            if (state.stepsTotal > 0) {
                uint64_t stepsMax = state.stepsAcc;
                uint64_t stepsRemaining = (state.stepsDone < stepsMax) ? (stepsMax - state.stepsDone) : 1;
                stepsToDo = std::min(stepsToDo, static_cast<uint32_t>((stepsRemaining > UINT32_MAX) ? UINT32_MAX : stepsRemaining));
            }
        }
        else if (state.currentState == State::Running) {
            UQ20x12 steps = (state.currentVelocity + 1000) / 1000;
            stepsToDo = steps.getInteger();
            if (state.stepsTotal > 0) {
                uint64_t stepsMax = state.stepsAcc + state.stepsConst;
                uint64_t stepsRemaining = (state.stepsDone < stepsMax) ? (stepsMax - state.stepsDone) : 1;
                stepsToDo = std::min(stepsToDo, static_cast<uint32_t>((stepsRemaining > UINT32_MAX) ? UINT32_MAX : stepsRemaining));
            }
        }
        else if (state.currentState == State::Decelerating) {
            if (state.deceleration == 0) {
                return 1;
            }

            UQ20x12 steps = (state.currentVelocity + state.deceleration) / state.deceleration;
            stepsToDo = steps.getInteger();
            if (state.stepsTotal > 0) {
                uint64_t stepsMax = state.stepsAcc + state.stepsConst + state.stepsDec;
                uint64_t stepsRemaining = (state.stepsDone < stepsMax) ? (stepsMax - state.stepsDone) : 1;
                stepsToDo = std::min(stepsToDo, static_cast<uint32_t>((stepsRemaining > UINT32_MAX) ? UINT32_MAX : stepsRemaining));
            }
        }
        else if (state.currentState == State::Stopped) {
            stepsToDo = 0;
        }
        else {
            LOG_WARN("Unexpected generator state: %d", static_cast<int>(state.currentState));
        }
        return stepsToDo;
   }

    void Generator::callbackOnStepDone(uint32_t stepsDone, uint32_t& stepsToDo, float& pulsePeriod_us, float& pulsePeriodIncrement_us, void* user_ctx) {
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
        pulsePeriod_us = self->computeStepPeriodUs(self->state_.currentVelocity);

        // Compute next step batch size
        stepsToDo = self->computeStepBatchSize(self->state_);
    }

} // namespace Generic
} // namespace Stepper
