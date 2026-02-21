#include "driver/mcpwm_types.h"
#ifndef STEPPER_DRIVER_MCPWM_H
#define STEPPER_DRIVER_MCPWM_H

#include <driver/mcpwm_prelude.h>
#include <freertos/queue.h>
#include "StepperDriver_Interface.h"


namespace Stepper
{
    class DriverMCPWM : public DriverInterface{
    public:
        DriverMCPWM() = delete;
        DriverMCPWM(int8_t enablePin, int8_t stepPin, int8_t directionPin);

        ~DriverMCPWM();

        void init() override;
        void start() override;
        void stop() override;
        void startOnce();
        void startSync();

        uint32_t stepsToDo {0};
        uint32_t onStepFull {0};
        uint32_t onStepEmpty {0};
        uint32_t onStepStop {0};

    private:
        inline uint32_t timerTicksFromNs(uint32_t timeNs, uint32_t timerResolutionHz = timerResolutionHz_) {
            float ticks = (float)timeNs * (float)timerResolutionHz / 2e9f;
            return static_cast<uint32_t>(ticks);
        };

        inline uint32_t timerTicksToNs(uint32_t ticks, uint32_t timerResolutionHz = timerResolutionHz_) {
            double timeNs = (float)ticks * 2e9f / (float)timerResolutionHz;
            return static_cast<uint32_t>(timeNs);
        };
        
        static void task(void* args);
        void update(uint32_t stepsNew, uint32_t pulsePeriodNew) override;
        static bool comperatorCallbackOnReach(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnEmpty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnStop(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);

        static constexpr uint32_t timerResolutionHz_ = 10'000'000;
        static constexpr const char* log_tag = "Driver";

        QueueHandle_t taskQueueHandle_ {nullptr};

        mcpwm_timer_handle_t stepTimerHandle_ {nullptr};
        mcpwm_oper_handle_t  operatorHandle_ {nullptr};
        mcpwm_cmpr_handle_t  comparatorHandle_ {nullptr};
        mcpwm_gen_handle_t   generatorHandle_ {nullptr};
    };
}

#endif //STEPPER_DRIVER_MCPWM_H