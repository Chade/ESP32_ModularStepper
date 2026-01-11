#include "StepperGenerator.h"
#include <math.h>
#include <algorithm>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Stepper {

    static const char* LOG_TAG = "GenSimple";

    Generator::Generator(Driver& driver) : m_driver(driver) {
        m_driver.start();
        createTimer(TimerType::Hardware);
    }

    Generator::~Generator() {
        stop();
        deleteTimer();
        m_driver.stop();
    }

    void Generator::createTimer(Generator::TimerType type) {
        if (type == Generator::TimerType::Software) {
            esp_timer_create_args_t args{};
            args.callback = &Generator::timerCallback;
            args.arg = this;
            args.dispatch_method = ESP_TIMER_TASK;
            args.name = "StepperGeneratorTimer";
            ESP_ERROR_CHECK(esp_timer_create(&args, &m_timer));
        }
        else if (type == Generator::TimerType::Hardware) {
            gptimer_config_t cfg{};
            cfg.clk_src = GPTIMER_CLK_SRC_APB;
            cfg.direction = GPTIMER_DIRECTION_UP;
            cfg.resolution_hz = k_timerResolutionHz; // 1 MHz
            ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &m_gptimer));

            gptimer_event_callbacks_t cbs{};
            cbs.on_alarm = &Generator::gptimerOnAlarm;
            ESP_ERROR_CHECK(gptimer_register_event_callbacks(m_gptimer, &cbs, this));

            ESP_ERROR_CHECK(gptimer_enable(m_gptimer));
        }
    }

    void Generator::stopTimer() {
        if (m_timer) {
            if (esp_timer_is_active(m_timer)) {
                esp_timer_stop(m_timer);
            }
        }

        if (m_gptimer) {
            if (gptimer_is_running(m_gptimer)) {
                gptimer_stop(m_gptimer);
            }
        }
    }

    void Generator::deleteTimer() {
        if (m_timer) {
            esp_timer_stop(m_timer);
            esp_timer_delete(m_timer);
            m_timer = nullptr;
        }

        if (m_gptimer) {
            gptimer_stop(m_gptimer);
            gptimer_disable(m_gptimer);
            gptimer_del_timer(m_gptimer);
            m_gptimer = nullptr;
        }
    }

    void Generator::setVelocity(float targetVelocity, float acceleration, float deceleration, Direction direction) {
        // Clamp to non-negative values
        m_targetVelocity = std::max(0.0f, targetVelocity);
        m_acceleration   = std::max(0.0f, acceleration);
        m_deceleration   = std::max(0.0f, deceleration);
        m_targetDirection = direction;

        if (m_currentVelocity <= 0.0f && m_targetVelocity > 0.0f) {
            m_currentVelocity = std::min(k_minVelocity, m_targetVelocity); // seed to avoid div by zero
        
            m_driver.setDirection(m_targetDirection);
            m_currentDirection = m_targetDirection;
        }

        if (!(m_state & State::RUNNING)) {
            m_state = State::RUNNING;
            // Re-arm timer with new period to adapt quickly
            uint64_t period = computePeriodUs(m_currentVelocity);
            scheduleNextStep(period);
        }
    }

    void Generator::moveSteps(uint64_t steps,
                                    float targetVelocity,
                                    float acceleration,
                                    float deceleration,
                                    Direction direction) {
        // Clamp inputs
        targetVelocity = std::max(0.0f, targetVelocity);
        acceleration   = std::max(0.0f, acceleration);
        deceleration   = std::max(0.0f, deceleration);

        m_totalSteps = steps;
        m_stepsDone = 0;
        
        // Compute ramp distribution (start from currentVelocity)
        // Steps to accelerate from currentVelocity to targetVelocity
        auto calcStepsNeeded = [](float v0, float v1, float a) -> uint64_t {
            if (a <= 0.0f) return 0;
            float dv = std::max(0.0f, v1 - v0);
            // s = 0.5 * dv^2 / a
            float s = 0.5f * dv * dv / a;
            return static_cast<uint32_t>(s);
        };
        
        uint32_t stepsAcc = calcStepsNeeded(m_currentVelocity, targetVelocity, acceleration);
        uint32_t stepsDec = calcStepsNeeded(targetVelocity, 0.0f, deceleration);

        if (stepsAcc + stepsDec <= m_totalSteps) {
            m_stepsAcc   = stepsAcc;
            m_stepsDec   = stepsDec;
            m_stepsConst = m_totalSteps - (stepsAcc + stepsDec);
        } else {
            // Triangular profile: scale acc/dec phases to fit total steps
            if (stepsAcc == 0 && stepsDec != 0) {
                m_stepsAcc = 0; m_stepsConst = 0; m_stepsDec = m_totalSteps;
            } else if (stepsAcc != 0 && stepsDec == 0) {
                m_stepsAcc = m_totalSteps; m_stepsConst = 0; m_stepsDec = 0;
            } else if (stepsAcc == 0 && stepsDec == 0) {
                m_stepsAcc = 0; m_stepsConst = m_totalSteps; m_stepsDec = 0;
            } else {
                float ratio = std::min(stepsAcc, stepsDec) / static_cast<float>(std::max(stepsAcc, stepsDec));
                m_stepsAcc = static_cast<uint64_t>(m_totalSteps * ratio);
                m_stepsDec = m_totalSteps - m_stepsAcc;
                m_stepsConst = 0;
            }
        }

        setVelocity(targetVelocity, acceleration, deceleration, direction);
    }

    void Generator::stop() {
        stopTimer();
        m_state = State::STOPPED;
        m_currentDirection = Direction::NEUTRAL;
        m_targetDirection = Direction::NEUTRAL;
        m_currentVelocity = 0.0f;
        m_targetVelocity = 0.0f;
        m_acceleration = 0.0f;
        m_deceleration = 0.0f;
        m_totalSteps = 0;
        m_stepsDone = 0;
        m_stepsAcc = m_stepsConst = m_stepsDec = 0;
    }

    void Generator::timerCallback(void* arg) {
        auto* self = static_cast<Generator*>(arg);

        if (!(self->m_state & State::RUNNING)) {
            return;
        }

        // Produce one step
        self->m_driver.doStep();

        // Advance internal state after step
        self->advanceStateAfterStep();

        // Schedule next step if still running
        if (self->m_state & State::RUNNING) {
            uint64_t period_us = self->computeStepPeriodUs(self->m_currentVelocity);
            self->scheduleNextStep(period_us);
        }
    }

    bool Generator::gptimerOnAlarm(gptimer_handle_t /*timer*/, const gptimer_alarm_event_data_t* /*edata*/, void* user_ctx) {
        auto* self = static_cast<Generator*>(user_ctx);
        
        bool ret = false;

        if (!(self->m_state & State::RUNNING)) {
            return;
        }

        // Produce one step via ISR-safe driver call
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        self->m_driver.doStepFromISR(&xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            //portYIELD_FROM_ISR();
            ret = true
        }

        // Advance internal state after step
        self->advanceStateAfterStep();

        // Schedule next step if still running
        if (self->m_state & State::RUNNING) {
            uint64_t period_us = self->computeStepPeriodUs(self->m_currentVelocity);
            self->scheduleNextStep(period_us);
        }
        else {
            // Stop timer if not running anymore
            gptimer_stop(self->m_gptimer);
        }

        return ret;
    }

    void Generator::rearmTimer(uint64_t period_us) {
        if (m_timer) {
            if (esp_timer_is_active(m_timer)) {
                esp_timer_stop(m_timer);
            }

            ESP_ERROR_CHECK(esp_timer_start_once(m_timer, period_us));
        }
        
        if (m_gptimer) {
            if (gptimer_is_running(m_gptimer)) {
                gptimer_stop(m_gptimer);
            }

            gptimer_alarm_config_t alarm{};
            alarm.reload_count = 0;
            alarm.alarm_count = period_us; // 1 tick per us
            alarm.flags.auto_reload_on_alarm = false;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(m_gptimer, &alarm));
            ESP_ERROR_CHECK(gptimer_start(m_gptimer));
        }
    }

    void Generator::scheduleNextStep(uint64_t period_us) {
        if (period_us == 0) {
            stopTimer();
            m_state = State::STOPPED;
            return;
        }
        
        rearmTimer(period_us);
    }

    uint64_t Generator::computeStepPeriodUs(float velocity) const {
        // Compute period from velocity
        if (velocity <= 0.0f) {
            return 0; // default 0 us when stopped
        }
        // period = 1e6 / v (microseconds per step)
        float p = 1000000.0f / velocity;

        // Clamp to a sane range
        p = std::max(static_cast<float>(m_driver.getMinPulsePeriodUs()), p);
        p = std::min(static_cast<float>(m_driver.getMaxPulsePeriodUs()), p);

        return static_cast<uint64_t>(p);
    }

    // uint64_t Generator::calcStepsNeeded(float deltaVelocity, float acceleration)
    // {
    //     /*
    //     Calculate the steps (s) needed to reach the desired velocity (v) with the given acceleration (a):

    //     s = 0.5 * a * t² = 0.5 * v² / a
    //     */
    //     if (acceleration == 0)
    //         return 0;

    //     return 0.5 * deltaVelocity * deltaVelocity / acceleration;
    // }

    // uint64_t Generator::calcStepsNeeded(float currentVelocity, float targetVelocity, float acceleration)
    // {
    //     return calcStepsNeeded(targetVelocity - currentVelocity, acceleration);
    // }

    // uint64_t Generator::calcStepPeriodInTicks(uint64_t steps, float acceleration, uint64_t ticksPerSecond)
    // {
    //     /*
    //     The time (t) to finish the entire movement is calculated as:

    //     t = v / a

    //     The velocity at a given step can be calculated as:

    //     v = sqrt(2*s*a)

    //     For a given v in steps/s the time between two steps is:

    //     t = 1 / v = 1 / sqrt(2*s*a)

    //     s = 0.5 * a * t²
    //     2*a*s = (a*t)² = v² = (ticks_per_s/ticks)²
    //     ticks = ticks_per_s / sqrt(2*a*s)
    //     ticks = ticks_per_s / sqrt(2) / sqrt(a*s)
    //     */
    //     if (acceleration == 0.0)
    //         return 0;

    //     if (steps == 0)
    //         return MAX_STEP_PERIOD_IN_TICKS;

    //     return ticksPerSecond / sqrt(2 * steps * acceleration);
    // }

    // uint64_t Generator::calcStepPeriodInTicks(float velocity, uint64_t ticksPerSecond)
    // {
    //     /*
    //     For a given v in steps/s the time between two steps is:

    //     t = 1 / v
    //     */

    //     if (velocity == 0.0)
    //         return MAX_STEP_PERIOD_IN_TICKS;

    //     return ticksPerSecond / velocity;
    // }

    bool Generator::advanceStateAfterStep() {
        
        if (m_steps == 0) {
            // Velocity mode
            if (m_targetVelocity <= 0.0f) {
                // Decelerate to stop
                if (m_currentVelocity > 0.0f) {
                    m_state = State::DECELERATING;
                    float dt = 1.0f / std::max(k_minVelocity, m_currentVelocity);
                    m_currentVelocity = std::max(0.0f, m_currentVelocity - m_deceleration * dt);
                }
                if (m_currentVelocity <= 0.0f) {
                    m_state = State::STOPPED;
                    stop();
                }
            } else {
                if (m_currentDirection != m_targetDirection) {
                    // Need to change direction: decelerate to stand-still first
                    if (m_currentVelocity > 0.0f) {
                        m_state = State::DECELERATING;
                        float dt = 1.0f / std::max(k_minVelocity, m_currentVelocity);
                        m_currentVelocity = std::max(0.0f, m_currentVelocity - m_deceleration * dt);
                    }
                    if (m_currentVelocity <= 0.0f) {
                        // Reached stand-still, change direction
                        m_state = State::RUNNING;
                        m_driver.setDirection(m_targetDirection);
                        m_currentDirection = m_targetDirection;
                    }
                }
                else {
                    // Move towards target velocity
                    if (m_currentVelocity < m_targetVelocity) {
                        m_state = State::ACCELERATING;
                        float dt = 1.0f / std::max(k_minVelocity, m_currentVelocity);
                        m_currentVelocity = std::min(m_targetVelocity, m_currentVelocity + m_acceleration * dt);
                    }
                    else if (m_currentVelocity > m_targetVelocity) {
                        m_state = State::DECELERATING;
                        float dt = 1.0f / std::max(k_minVelocity, m_currentVelocity);
                        m_currentVelocity = std::max(m_targetVelocity, m_currentVelocity - m_deceleration * dt);
                    }
                    else {
                        m_state = State::RUNNING;
                    }
                }
            }
        }
        else {
            // Distance mode
            m_stepsDone++;

            // Determine phase by stepsDone
            if (m_stepsDone <= m_stepsAcc) {
                // Accelerating: v^2 = v0^2 + 2 a s; step-by-step approx
                m_state = State::ACCELERATING;
                m_currentVelocity = std::min(m_targetVelocity, m_currentVelocity + m_acceleration * dt);
            } else if (m_stepsDone <= (m_stepsAcc + m_stepsConst)) {
                // Constant
                m_state = State::RUNNING;
                m_currentVelocity = m_targetVelocity;
            } else if (m_stepsDone <= (m_stepsAcc + m_stepsConst + m_stepsDec)) {
                // Decelerating
                m_state = State::DECELERATING;
                m_currentVelocity = std::max(0.0f, m_currentVelocity - m_deceleration * dt);
            }

            if (m_stepsDone >= m_totalSteps) {
                m_state = State::STOPPED;
                stop();
            }
        }
    }

} // namespace Stepper
