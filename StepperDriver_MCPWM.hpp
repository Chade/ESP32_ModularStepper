#ifndef STEPPER_DRIVER_MCPWM_HPP
#define STEPPER_DRIVER_MCPWM_HPP

#include "driver/mcpwm_types.h"
#include <driver/mcpwm_prelude.h>
#include "StepperDriver_Base.hpp"


namespace Stepper
{
    class DriverMCPWM : public DriverBase{
    public:
        DriverMCPWM() = delete;
        DriverMCPWM(int8_t enablePin, int8_t stepPin, int8_t directionPin, uint8_t microsteps = 1);

        ~DriverMCPWM();

        void init() override;
        void start() override;
        void stop() override;
        void startOnce();
        void startSync();

    private:
        inline uint32_t timerTicksFromUs(float timeUs) {
            constexpr uint8_t scaleFixed = 12;

            uint32_t timeFixed = static_cast<uint32_t>(timeUs * (1 << scaleFixed));
                
            return (timeFixed >> (scaleFixed - timerScaleShift_ + timerBaseShift_));
        };

        inline uint32_t timerTicksToUs(uint32_t ticks) {
            if (timerScaleShift_ > timerBaseShift_)
                return ticks >> (timerScaleShift_ - timerBaseShift_);
            return ticks << (timerBaseShift_ - timerScaleShift_);
        };

        
        // Update function will be called periodically from the underlying task
        // with the number of steps done since last update and the new pulse period to apply.
        void update(uint32_t stepsNew, float pulsePeriodNew_us) override;

        static bool comperatorCallbackOnReach(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnEmpty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);
        static bool stepTimerCallbackOnStop(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);

        const uint8_t timerBaseShift_ = 2;
        const uint32_t timerBaseResolution_ = 250'000;
        const uint8_t timerScaleShift_;
        const uint32_t timerResolutionHz_;
        static constexpr const char* log_tag = "Driver";

        mcpwm_timer_handle_t stepTimerHandle_ {nullptr};
        mcpwm_oper_handle_t  operatorHandle_ {nullptr};
        mcpwm_cmpr_handle_t  comparatorHandle_ {nullptr};
        mcpwm_gen_handle_t   generatorHandle_ {nullptr};
    };
}

#endif //STEPPER_DRIVER_MCPWM_HPP