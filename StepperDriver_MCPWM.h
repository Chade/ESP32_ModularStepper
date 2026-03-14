#include "driver/mcpwm_types.h"
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

        void init() override;
        void start() override;
        void stop() override;
        void startOnce();
        void startSync();


        uint32_t onStepFull {0};
        uint32_t onStepEmpty {0};
        uint32_t onStepStop {0};

        uint32_t onUpdateFull {0};
        uint32_t onUpdateEmpty {0};
        uint32_t onUpdateStop {0};

    private:
        inline uint32_t timerTicksFromUs(float timeUs, uint32_t timerResolutionHz = timerResolutionHz_) {
            float ticks = timeUs * static_cast<float>(timerResolutionHz) / 2e6f;
            return static_cast<uint32_t>(ticks);
        };

        inline float timerTicksToUs(uint32_t ticks, uint32_t timerResolutionHz = timerResolutionHz_) {
            return static_cast<float>(ticks) * 2e6f / static_cast<float>(timerResolutionHz);
        };

        inline uint32_t timerTicksFromNs(float timeNs, uint32_t timerResolutionHz = timerResolutionHz_) {
            float ticks = timeNs * static_cast<float>(timerResolutionHz) / 2e9f;
            return static_cast<uint32_t>(ticks);
        };

        inline float timerTicksToNs(uint32_t ticks, uint32_t timerResolutionHz = timerResolutionHz_) {
            return static_cast<float>(ticks) * 2e9f / static_cast<float>(timerResolutionHz);
        };
        
        void update(uint32_t stepsNew, float pulsePeriodNew) override;
        static bool comperatorCallbackOnReach(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnEmpty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnStop(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);

        static constexpr uint32_t timerResolutionHz_ = 10'000'000;
        static constexpr const char* log_tag = "Driver";

        mcpwm_timer_handle_t stepTimerHandle_ {nullptr};
        mcpwm_oper_handle_t  operatorHandle_ {nullptr};
        mcpwm_cmpr_handle_t  comparatorHandle_ {nullptr};
        mcpwm_gen_handle_t   generatorHandle_ {nullptr};
    };
}

#endif //STEPPER_DRIVER_MCPWM_H