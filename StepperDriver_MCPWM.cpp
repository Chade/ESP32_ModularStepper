#include "driver/mcpwm_gen.h"
#include "hal/mcpwm_types.h"
#include "driver/mcpwm_sync.h"
#include "driver/mcpwm_timer.h"
#include "StepperDriver_MCPWM.hpp"
#include "StepperLog.hpp"

namespace Stepper {
    DriverMCPWM::DriverMCPWM(int8_t enablePin, int8_t stepPin, int8_t directionPin, uint8_t microsteps)
            : DriverBase(enablePin, stepPin, directionPin, microsteps),
              timerScaleShift_((microsteps != 0) ? 32 - __builtin_clz(microsteps) : 0),
              timerResolutionHz_(timerBaseResolution_ << timerScaleShift_) {

        // Calc the min pulse period based on the timer resolution
        minPulsePeriod_us_ = 1'000'000.0f / timerResolutionHz_;
        // Calc the max pulse period based on the timer resolution and max value of the timer counter (16 bit)
        maxPulsePeriod_us_ = timerTicksToUs(UINT16_MAX);
    }

    DriverMCPWM::~DriverMCPWM() {

    }

    void DriverMCPWM::update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew_us) {
        ESP_ERROR_CHECK(mcpwm_timer_set_period(stepTimerHandle_, timerTicksFromUs(pulsePeriodNew_us)));
    }

    bool IRAM_ATTR DriverMCPWM::comperatorCallbackOnReach(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t* edata, void* user_ctx) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        self->doStepFromISR(&xHigherPriorityTaskWoken, true);
        return xHigherPriorityTaskWoken;
    }

    bool IRAM_ATTR DriverMCPWM::stepTimerCallbackOnFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        return false;
    }

    bool IRAM_ATTR DriverMCPWM::stepTimerCallbackOnEmpty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        return false;
    }

    bool IRAM_ATTR DriverMCPWM::stepTimerCallbackOnStop(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        mcpwm_generator_set_force_level(self->generatorHandle_, self->pinStep_.getLevelDisabled(), false);
        self->mcpwmTimerRunning_ = false;
        return false;
    }

    void DriverMCPWM::init() {
        mcpwm_timer_config_t stepTimerConfig;
        stepTimerConfig.group_id = 0;
        stepTimerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        stepTimerConfig.resolution_hz = timerResolutionHz_;
        stepTimerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        stepTimerConfig.period_ticks = timerTicksFromUs(maxPulsePeriod_us_);
        stepTimerConfig.intr_priority = 0;
        stepTimerConfig.flags.update_period_on_empty = 1;
        stepTimerConfig.flags.update_period_on_sync = 0;
        stepTimerConfig.flags.allow_pd = 0;

        ESP_ERROR_CHECK(mcpwm_new_timer(&stepTimerConfig, &stepTimerHandle_));

        mcpwm_timer_event_callbacks_t stepTimerCallbacks;
        stepTimerCallbacks.on_full = NULL;
        stepTimerCallbacks.on_empty = NULL;
        stepTimerCallbacks.on_stop = DriverMCPWM::stepTimerCallbackOnStop;

        ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(stepTimerHandle_, &stepTimerCallbacks, this));

        mcpwm_operator_config_t operatorConfig;
        operatorConfig.group_id = 0;
        operatorConfig.intr_priority = 0;
        operatorConfig.flags.update_gen_action_on_tez = 1;
        operatorConfig.flags.update_gen_action_on_tep = 0;
        operatorConfig.flags.update_gen_action_on_sync = 0;
        operatorConfig.flags.update_dead_time_on_tez = 0;
        operatorConfig.flags.update_dead_time_on_tep = 0;
        operatorConfig.flags.update_dead_time_on_sync = 0;

        ESP_ERROR_CHECK(mcpwm_new_operator(&operatorConfig, &operatorHandle_));

        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operatorHandle_, stepTimerHandle_));

        mcpwm_comparator_config_t comparatorConfig;
        comparatorConfig.intr_priority = 0;
        comparatorConfig.flags.update_cmp_on_tez = 1;
        comparatorConfig.flags.update_cmp_on_tep = 0;
        comparatorConfig.flags.update_cmp_on_sync = 0;

        ESP_ERROR_CHECK(mcpwm_new_comparator(operatorHandle_, &comparatorConfig, &comparatorHandle_));

        mcpwm_comparator_event_callbacks_t comperatorCallbacks;
        comperatorCallbacks.on_reach = &DriverMCPWM::comperatorCallbackOnReach;
        ESP_ERROR_CHECK(mcpwm_comparator_register_event_callbacks(comparatorHandle_, &comperatorCallbacks, this));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorHandle_, std::max(timerTicksFromUs(minPulseWidthHigh_us_), timerTicksFromUs(minPulsePeriod_us_))));

        mcpwm_generator_config_t generatorConfig;
        generatorConfig.gen_gpio_num = pinStep_.getPin();
        generatorConfig.flags.invert_pwm = pinStep_.getLevelDisabled();
        generatorConfig.flags.io_loop_back = 0;
        generatorConfig.flags.io_od_mode = 0;
        generatorConfig.flags.pull_up = 0;
        generatorConfig.flags.pull_down = 1;

        ESP_ERROR_CHECK(mcpwm_new_generator(operatorHandle_, &generatorConfig, &generatorHandle_));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorHandle_,
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generatorHandle_,
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorHandle_, MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_timer_enable(stepTimerHandle_));
    }

    void DriverMCPWM::start() {
        reset();
        ESP_ERROR_CHECK(mcpwm_timer_set_period(stepTimerHandle_, timerTicksFromUs(pulsePeriod_us_)));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_START_NO_STOP));
        mcpwmTimerRunning_ = true;
        ESP_LOGI(log_tag, "MCPWM timer started");
    }

    void DriverMCPWM::startOnce() {
        reset();
        ESP_ERROR_CHECK(mcpwm_timer_set_period(stepTimerHandle_, timerTicksFromUs(pulsePeriod_us_)));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_START_STOP_FULL));
        //ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_START_STOP_EMPTY));
        mcpwmTimerRunning_ = true;
        ESP_LOGI(log_tag, "MCPWM timer started once");
    }

    void DriverMCPWM::stop() {
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_STOP_FULL));
        mcpwmTimerRunning_ = false;
        ESP_LOGI(log_tag, "MCPWM timer stopped");
    }

    bool DriverMCPWM::isRunning() {
        return mcpwmTimerRunning_;
    }
}