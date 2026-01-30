#ifndef STEPPER_DRIVER_MCPWM_H
#define STEPPER_DRIVER_MCPWM_H

#include <driver/mcpwm_prelude.h>
#include "StepperDriver_Interface.h"


namespace Stepper
{
    class DriverMCPWM : public DriverInterface{
    public:
        DriverMCPWM() = delete;
        DriverMCPWM(int8_t enablePin, int8_t stepPin, int8_t directionPin);

        ~DriverMCPWM();

        void init();
        void start();
        void startOnce();
        void stop();

        uint32_t onFull {0};
        uint32_t onEmpty {0};
        uint32_t onStop {0};
        uint32_t onCmpr {0};

    private:
        bool taskLoop(uint32_t notificationValue);
        static bool comperatorCallbackOnReach(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t* edata, void* user_ctx);
        static bool timerCallbackOnFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool timerCallbackOnEmpty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool timerCallbackOnStop(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);

        inline uint32_t timerTicksFromNs(uint32_t timeNs) {
            return timeNs * timerResolutionHz_ / 1e9f;
        };

        inline uint32_t timerTicksToNs(uint32_t ticks) {
            return ticks * 1e9f / timerResolutionHz_;
        };

        static constexpr uint32_t timerResolutionHz_ = 10'000'000;

        mcpwm_timer_handle_t timerHandle_ {nullptr};
        mcpwm_oper_handle_t  operatorHandle_ {nullptr};
        mcpwm_cmpr_handle_t  comparatorHandle_ {nullptr};
        mcpwm_gen_handle_t   generatorHandle_ {nullptr};
    };
}

#endif //STEPPER_DRIVER_MCPWM_H