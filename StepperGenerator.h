#ifndef STEPPER_GENERATOR_H
#define STEPPER_GENERATOR_H

#include "StepperDriver.h"
#include <driver/gptimer.h>
#include <esp_timer.h>
#include <stdint.h>

namespace Stepper
{

    class Generator
    {
    public:
        Generator() = delete
        Generator(Driver& driver);
        ~Generator();

        // Velocity mode: ramp to target velocity (steps/s) using given accel/decel (steps/s^2).
        // Call again with targetVelocity = 0 to decelerate to stop.
        void setVelocity(float targetVelocity, float acceleration, float deceleration, Direction direction);

        // Distance mode: perform 'steps' at up to targetVelocity with accel/decel ramp (steps/s^2).
        // Returns immediately; generator schedules steps until done.
        void moveSteps(uint64_t steps,
                       float targetVelocity,
                       float acceleration,
                       float deceleration,
                       Direction direction);

        // Stop any ongoing generation.
        void stop();

        bool getState() const { return m_state; }

    private:
        enum class TimerType
        {
            Software,
            Hardware
        };

        enum class State : uint8_t
        {
            UNDEFINED = 0,
            RUNNING = 1,
            ACCELERATING = 3,
            DECELERATING = 5,
            PAUSED = 16,
            STOPPED = 32,
            INHIBITED = 96,
            EMERGENCYSTOP = 224
        };

        void createTimer(TimerType type);
        void stopTimer();
        void deleteTimer();
        void rearmTimer(uint64_t period_us);
        static void timerCallback(void *arg);
        static bool gptimerOnAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

        void scheduleNextStep(uint64_t period_us);
        uint64_t computePeriodUsForCurrentState() const;
        void advanceStateAfterStep();

        // Shared state
        Driver& m_driver;
        esp_timer_handle_t m_timer = nullptr;
        gptimer_handle_t m_gptimer = nullptr;
        
        State m_state = State::UNDEFINED;

        // Kinematic state
        float m_currentVelocity = 0.0f; // steps/s
        float m_targetVelocity = 0.0f;  // steps/s
        float m_acceleration = 0.0f;    // steps/s^2
        float m_deceleration = 0.0f;    // steps/s^2

        Direction m_currentDirection = Direction::NEUTRAL; // current direction
        Direction m_targetDirection = Direction::NEUTRAL;  // target direction

        // Distance mode state
        uint64_t m_totalSteps = 0; // total steps requested
        uint64_t m_stepsDone = 0;  // steps already executed
        uint64_t m_stepsAcc = 0;   // steps in acceleration phase
        uint64_t m_stepsConst = 0; // steps in constant velocity phase
        uint64_t m_stepsDec = 0;   // steps in deceleration phase

        static constexpr uint32_t k_timerResolutionHz = 1000000; // 1 MHz resolution (1 tick = 1 us)
        static constexpr float k_minVelocity = 1.0f;  // steps/s

} // namespace Stepper

#endif // STEPPER_GENERATOR_H