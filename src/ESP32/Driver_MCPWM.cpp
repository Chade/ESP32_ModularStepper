#include <hal/mcpwm_types.h>
#include <driver/mcpwm_gen.h>
#include <driver/mcpwm_sync.h>
#include <driver/mcpwm_timer.h>
#include <soc/soc_caps.h>

#include "Driver_MCPWM.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {
namespace MCPWM {
    Driver::Driver(Generic::Pin* enablePin, Generic::Pin& stepPin, Generic::Pin& directionPin, uint8_t microsteps, int groupId)
            : Stepper::ESP32::Driver(enablePin, stepPin, directionPin, microsteps),
                timerScaleShift_((microsteps != 0) ? 32 - __builtin_clz(microsteps) : 0),
                timerResolutionHz_(timerBaseResolution_ << timerScaleShift_),
                requestedGroupId_(groupId) {

        // Calc the min pulse period based on the timer resolution
        minPulsePeriod_us_ = 1'000'000.0f / timerResolutionHz_;
        // Calc the max pulse period based on the timer resolution and max value of the timer counter (16 bit)
        maxPulsePeriod_us_ = timerTicksToUs(UINT16_MAX);
    }

    Driver::Driver(Generic::Pin& enablePin, Generic::Pin& stepPin, Generic::Pin& directionPin, uint8_t microsteps, int groupId)
            : Driver(&enablePin, stepPin, directionPin, microsteps, groupId) {
    }

    Driver::~Driver() {
        deinit();
    }

    void Driver::deinit() {
        if (stepTimerHandle_ != nullptr) {
            if (mcpwmTimerRunning_) {
                mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_STOP_FULL);
                mcpwmTimerRunning_ = false;
            }
            if (timerEnabled_) {
                mcpwm_timer_disable(stepTimerHandle_);
                timerEnabled_ = false;
            }
        }

        if (generatorHandle_ != nullptr) {
            mcpwm_del_generator(generatorHandle_);
            generatorHandle_ = nullptr;
        }
        if (comparatorHandle_ != nullptr) {
            mcpwm_del_comparator(comparatorHandle_);
            comparatorHandle_ = nullptr;
        }
        if (operatorHandle_ != nullptr) {
            mcpwm_del_operator(operatorHandle_);
            operatorHandle_ = nullptr;
        }
        if (stepTimerHandle_ != nullptr) {
            mcpwm_del_timer(stepTimerHandle_);
            stepTimerHandle_ = nullptr;
        }

        activeGroupId_ = -1;
    }

    void Driver::update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew_us, float pulsePeriodIncrement_us) {
        if (stepTimerHandle_ == nullptr) {
            return;
        }
        ESP_ERROR_CHECK(mcpwm_timer_set_period(stepTimerHandle_, timerTicksFromUs(pulsePeriodNew_us)));
    }

    bool IRAM_ATTR Driver::comperatorCallbackOnReach(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t* edata, void* user_ctx) {
        Driver* self = static_cast<Driver*>(user_ctx);
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        self->notifyStepDoneFromISR(&xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken;
    }

    bool IRAM_ATTR Driver::stepTimerCallbackOnFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        Driver* self = static_cast<Driver*>(user_ctx);
        return false;
    }

    bool IRAM_ATTR Driver::stepTimerCallbackOnEmpty(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        Driver* self = static_cast<Driver*>(user_ctx);
        return false;
    }

    bool IRAM_ATTR Driver::stepTimerCallbackOnStop(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx) {
        Driver* self = static_cast<Driver*>(user_ctx);
        mcpwm_generator_set_force_level(self->generatorHandle_, self->pinStep_.getLevelDisabled(), false);
        self->mcpwmTimerRunning_ = false;
        return false;
    }

    bool Driver::init() {
        if (stepTimerHandle_ != nullptr && taskHandle_ != nullptr) {
            return true;
        }

        if (!ESP32::Driver::init()) {
            return false;
        }

        if (requestedGroupId_ >= SOC_MCPWM_GROUPS) {
            LOG_ERROR("Invalid MCPWM group id %d (available groups: 0..%d)", requestedGroupId_, SOC_MCPWM_GROUPS - 1);
            return false;
        }

        const int groupStart = (requestedGroupId_ >= 0) ? requestedGroupId_ : 0;
        const int groupEnd   = (requestedGroupId_ >= 0) ? requestedGroupId_ : (SOC_MCPWM_GROUPS - 1);

        for (int group = groupStart; group <= groupEnd; ++group) {
            mcpwm_timer_config_t stepTimerConfig = {};
            stepTimerConfig.group_id = group;
            stepTimerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
            stepTimerConfig.resolution_hz = timerResolutionHz_;
            stepTimerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
            stepTimerConfig.period_ticks = timerTicksFromUs(maxPulsePeriod_us_);
            stepTimerConfig.intr_priority = 0;
            stepTimerConfig.flags.update_period_on_empty = 1;
            stepTimerConfig.flags.update_period_on_sync = 0;
            stepTimerConfig.flags.allow_pd = 0;

            esp_err_t err = mcpwm_new_timer(&stepTimerConfig, &stepTimerHandle_);
            if (err == ESP_ERR_NOT_FOUND) {
                stepTimerHandle_ = nullptr;
                continue;
            }
            if (err != ESP_OK) {
                LOG_ERROR("Failed to allocate MCPWM timer in group %d (err=0x%x)", group, err);
                deinit();
                return false;
            }

            mcpwm_operator_config_t operatorConfig = {};
            operatorConfig.group_id = group;
            operatorConfig.intr_priority = 0;
            operatorConfig.flags.update_gen_action_on_tez = 1;
            operatorConfig.flags.update_gen_action_on_tep = 0;
            operatorConfig.flags.update_gen_action_on_sync = 0;
            operatorConfig.flags.update_dead_time_on_tez = 0;
            operatorConfig.flags.update_dead_time_on_tep = 0;
            operatorConfig.flags.update_dead_time_on_sync = 0;

            err = mcpwm_new_operator(&operatorConfig, &operatorHandle_);
            if (err == ESP_ERR_NOT_FOUND) {
                operatorHandle_ = nullptr;
                mcpwm_del_timer(stepTimerHandle_);
                stepTimerHandle_ = nullptr;
                continue;
            }
            if (err != ESP_OK) {
                LOG_ERROR("Failed to allocate MCPWM operator in group %d (err=0x%x)", group, err);
                deinit();
                return false;
            }

            activeGroupId_ = group;
            break;
        }

        if (activeGroupId_ < 0 || stepTimerHandle_ == nullptr || operatorHandle_ == nullptr) {
            LOG_ERROR("No free MCPWM timer/operator available in any group");
            deinit();
            return false;
        }

        mcpwm_timer_event_callbacks_t stepTimerCallbacks = {};
        stepTimerCallbacks.on_full = NULL;
        stepTimerCallbacks.on_empty = NULL;
        stepTimerCallbacks.on_stop = Driver::stepTimerCallbackOnStop;

        if (mcpwm_timer_register_event_callbacks(stepTimerHandle_, &stepTimerCallbacks, this) != ESP_OK ||
            mcpwm_operator_connect_timer(operatorHandle_, stepTimerHandle_) != ESP_OK) {
            LOG_ERROR("Failed to bind MCPWM timer callbacks/operator");
            deinit();
            return false;
        }

        mcpwm_comparator_config_t comparatorConfig = {};
        comparatorConfig.intr_priority = 0;
        comparatorConfig.flags.update_cmp_on_tez = 1;
        comparatorConfig.flags.update_cmp_on_tep = 0;
        comparatorConfig.flags.update_cmp_on_sync = 0;

        if (mcpwm_new_comparator(operatorHandle_, &comparatorConfig, &comparatorHandle_) != ESP_OK) {
            LOG_ERROR("Failed to allocate MCPWM comparator");
            deinit();
            return false;
        }

        mcpwm_comparator_event_callbacks_t comperatorCallbacks = {};
        comperatorCallbacks.on_reach = &Driver::comperatorCallbackOnReach;
        if (mcpwm_comparator_register_event_callbacks(comparatorHandle_, &comperatorCallbacks, this) != ESP_OK ||
            mcpwm_comparator_set_compare_value(comparatorHandle_, timerTicksFromUs(std::max(minPulseWidthHigh_us_, minPulsePeriod_us_))) != ESP_OK) {
            LOG_ERROR("Failed to configure MCPWM comparator");
            deinit();
            return false;
        }

        mcpwm_generator_config_t generatorConfig = {};
        generatorConfig.gen_gpio_num = pinStep_.getPin();
        generatorConfig.flags.invert_pwm = pinStep_.getLevelDisabled();
        generatorConfig.flags.io_loop_back = 0;
        generatorConfig.flags.io_od_mode = 0;
        generatorConfig.flags.pull_up = 0;
        generatorConfig.flags.pull_down = 1;

        if (mcpwm_new_generator(operatorHandle_, &generatorConfig, &generatorHandle_) != ESP_OK ||
            mcpwm_generator_set_action_on_timer_event(generatorHandle_,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)) != ESP_OK ||
            mcpwm_generator_set_action_on_compare_event(generatorHandle_,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorHandle_, MCPWM_GEN_ACTION_LOW)) != ESP_OK ||
            mcpwm_timer_enable(stepTimerHandle_) != ESP_OK) {
            LOG_ERROR("Failed to configure/enable MCPWM generator/timer");
            deinit();
            return false;
        }

        timerEnabled_ = true;
        return true;
    }

    void Driver::start() {
        if (stepTimerHandle_ == nullptr) {
            return;
        }
        ESP_ERROR_CHECK(mcpwm_timer_set_period(stepTimerHandle_, timerTicksFromUs(pulsePeriod_us_)));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_START_NO_STOP));
        mcpwmTimerRunning_ = true;
        LOG_INFO("MCPWM timer started");
    }

    void Driver::startOnce() {
        if (stepTimerHandle_ == nullptr) {
            return;
        }
        ESP_ERROR_CHECK(mcpwm_timer_set_period(stepTimerHandle_, timerTicksFromUs(pulsePeriod_us_)));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_START_STOP_FULL));
        //ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_START_STOP_EMPTY));
        mcpwmTimerRunning_ = true;
        LOG_INFO("MCPWM timer started once");
    }

    void Driver::stop() {
        if (stepTimerHandle_ == nullptr) {
            return;
        }
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(stepTimerHandle_, MCPWM_TIMER_STOP_FULL));
        mcpwmTimerRunning_ = false;
        LOG_INFO("MCPWM timer stopped");
    }

    bool Driver::isRunning() {
        return mcpwmTimerRunning_;
    }
} // namespace MCPWM
} // namespace ESP32
} // namespace Stepper
