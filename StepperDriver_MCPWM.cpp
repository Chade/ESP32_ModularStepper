#include "hal/mcpwm_types.h"
#include "StepperDriver_MCPWM.h"

namespace Stepper {
    DriverMCPWM::DriverMCPWM(int8_t enablePin, int8_t stepPin, int8_t directionPin)
            : DriverInterface(enablePin, stepPin, directionPin) {
    }

    DriverMCPWM::~DriverMCPWM() {

    }

    bool DriverMCPWM::taskLoop(uint32_t notificationValue) {
        return false;
    }

    bool IRAM_ATTR DriverMCPWM::comperatorCallbackOnReach(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t* edata, void* user_ctx) {
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        self->onCmpr += 1;
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        self->doStepFromISR(&xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken;
    }

    bool IRAM_ATTR DriverMCPWM::timerCallbackOnFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        self->onFull += 1;
        return false;
    }

    bool IRAM_ATTR DriverMCPWM::timerCallbackOnEmpty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        self->onEmpty += 1;
        return false;
    }

    bool IRAM_ATTR DriverMCPWM::timerCallbackOnStop(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        DriverMCPWM* self = static_cast<DriverMCPWM*>(user_ctx);
        self->onStop += 1;
        return false;
    }

    void DriverMCPWM::init() {
        mcpwm_timer_config_t timerConfig;
        timerConfig.group_id = 0;
        timerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timerConfig.resolution_hz = timerResolutionHz_;
        timerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        timerConfig.period_ticks = 60000;
        timerConfig.intr_priority = 0;
        timerConfig.flags.update_period_on_empty = 1;
        timerConfig.flags.update_period_on_sync = 0;
        timerConfig.flags.allow_pd = 0;

        ESP_ERROR_CHECK(mcpwm_new_timer(&timerConfig, &timerHandle_));

        mcpwm_timer_event_callbacks_t timerCallbacks;
        timerCallbacks.on_full = DriverMCPWM::timerCallbackOnFull;
        timerCallbacks.on_empty = DriverMCPWM::timerCallbackOnEmpty;
        timerCallbacks.on_stop = DriverMCPWM::timerCallbackOnStop;

        ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timerHandle_, &timerCallbacks, this));

        mcpwm_operator_config_t operatorConfig;
        operatorConfig.group_id = 0;
        operatorConfig.intr_priority = 0;
        operatorConfig.flags.update_gen_action_on_tez = 0;
        operatorConfig.flags.update_gen_action_on_tep = 1;
        operatorConfig.flags.update_gen_action_on_sync = 0;
        operatorConfig.flags.update_dead_time_on_tez = 0;
        operatorConfig.flags.update_dead_time_on_tep = 0;
        operatorConfig.flags.update_dead_time_on_sync = 0;

        ESP_ERROR_CHECK(mcpwm_new_operator(&operatorConfig, &operatorHandle_));

        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operatorHandle_, timerHandle_));

        mcpwm_comparator_config_t comparatorConfig;
        comparatorConfig.intr_priority = 0;
        comparatorConfig.flags.update_cmp_on_tez = 0;
        comparatorConfig.flags.update_cmp_on_tep = 1;
        comparatorConfig.flags.update_cmp_on_sync = 0;

        ESP_ERROR_CHECK(mcpwm_new_comparator(operatorHandle_, &comparatorConfig, &comparatorHandle_));

        mcpwm_comparator_event_callbacks_t comperatorCallbacks;
        comperatorCallbacks.on_reach = &DriverMCPWM::comperatorCallbackOnReach;
        ESP_ERROR_CHECK(mcpwm_comparator_register_event_callbacks(comparatorHandle_, &comperatorCallbacks, this));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorHandle_, timerTicksFromNs(stepPulseWidthHigh_ns_)));

        mcpwm_generator_config_t generatorConfig;
        generatorConfig.gen_gpio_num = 25;
        generatorConfig.flags.invert_pwm = 0;
        generatorConfig.flags.io_loop_back = 0;
        generatorConfig.flags.io_od_mode = 0;
        generatorConfig.flags.pull_up = 0;
        generatorConfig.flags.pull_down = 1;

        ESP_ERROR_CHECK(mcpwm_new_generator(operatorHandle_, &generatorConfig, &generatorHandle_));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorHandle_,
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generatorHandle_,
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorHandle_, MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_timer_enable(timerHandle_));
    }

    void DriverMCPWM::start() {
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerHandle_, MCPWM_TIMER_START_NO_STOP));
    }

    void DriverMCPWM::startOnce() {
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerHandle_, MCPWM_TIMER_START_STOP_FULL));
        //ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerHandle_, MCPWM_TIMER_START_STOP_EMPTY));
    }

    void DriverMCPWM::stop() {
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerHandle_, MCPWM_TIMER_STOP_FULL));
    }
}