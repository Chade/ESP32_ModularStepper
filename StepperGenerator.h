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
        Generator() = delete;
        Generator(Driver& driver);
        ~Generator();

        struct GeneratorTask {
            uint64_t steps = 0;
            float velocity = 0.0f;
            float acceleration = 0.0f;
            float deceleration = 0.0f;
            Direction direction = Direction::NEUTRAL;
        };

        bool run(const GeneratorTask& task, uint32_t waitFor_ms = portMAX_DELAY);

        // Velocity mode: Ramp to target velocity (steps/s) using given accel/decel (steps/s^2).
        // Call again with targetVelocity = 0 to decelerate to stop.
        bool run(float targetVelocity, float acceleration, float deceleration, Direction direction, uint32_t waitFor_ms = portMAX_DELAY);

        // Steps mode: Perform 'steps' at up to targetVelocity with accel/decel ramp (steps/s^2).
        // Returns immediately; generator schedules steps until done.
        bool run(uint64_t steps, float targetVelocity, float acceleration, float deceleration, Direction direction, uint32_t waitFor_ms = portMAX_DELAY);

        void start();
        void stop();

        State getState() const;
        void resetState();

    private:
        enum class TimerType
        {
            Software,
            Hardware
        };

        struct GeneratorState {
            State state = State::UNDEFINED; // movement state

            Direction currentDirection = Direction::NEUTRAL; // current direction
            Direction targetDirection  = Direction::NEUTRAL; // target direction

            bool doDirectionChange = false; // request direction change

            // Kinematic state
            float currentVelocity = 0.0f; // steps/s
            float targetVelocity  = 0.0f; // steps/s
            float acceleration    = 0.0f; // steps/s^2
            float deceleration    = 0.0f; // steps/s^2

            // Distance mode state
            uint64_t stepsTotal = 0; // total steps requested
            uint64_t stepsDone  = 0; // steps already executed
            uint64_t stepsAcc   = 0; // steps in acceleration phase
            uint64_t stepsConst = 0; // steps in constant velocity phase
            uint64_t stepsDec   = 0; // steps in deceleration phase
        } m_state;

        struct StepperTask {
            uint64_t period_us = 0;
        };

        void createTimer(TimerType type);
        void stopTimer();
        void deleteTimer();
        void rearmTimer(uint64_t period_us);
        static void timerCallback(void *arg);
        static bool gptimerOnAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
        static void generatorTask(void *args);

        void scheduleNextStep(uint64_t period_us);
        uint64_t computeStepPeriodUs(float velocity) const;
        bool initializeStateBeforeStep(const GeneratorTask&, GeneratorState& state);
        bool advanceStateAfterStep(GeneratorState& state, float minVelocity = 10.0f);

        Driver& m_driver;
        esp_timer_handle_t m_timer = nullptr;
        gptimer_handle_t m_gptimer = nullptr;

        TaskHandle_t m_taskHandle = nullptr;
        QueueHandle_t m_taskQueueHandle = nullptr;
        QueueHandle_t m_stepQueueHandle = nullptr;
 
        static constexpr uint32_t k_timerResolutionHz = 1000000; // 1 MHz resolution (1 tick = 1 us)
        static constexpr uint32_t k_taskQueueSize = 10;
        static constexpr float k_minVelocity = 1.0f;  // steps/s
        static constexpr const char* log_tag = "Generator";
    };
} // namespace Stepper

#endif // STEPPER_GENERATOR_H