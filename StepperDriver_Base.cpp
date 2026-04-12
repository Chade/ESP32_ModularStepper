#include "esp32-hal-gpio.h"
#include "esp_task_wdt.h"
#include "StepperDriver_Base.hpp"

namespace Stepper {
    DriverBase::DriverBase(int8_t enablePin, int8_t stepPin, int8_t directionPin, uint8_t microsteps)
            : pinEnable_(enablePin), pinStep_(stepPin), pinDirection_(directionPin), microsteps_(microsteps) {
        
        // Create step task
        xTaskCreatePinnedToCore(
            DriverBase::task,    /* Task function */
            "StepTask",               /* Task name (max. 16 characters by default) */
            8192,                     /* Stack size in bytes */
            this,                     /* Parameter passed as input of the task */
            10,                       /* Task priority */
            &taskHandle_,             /* Task handle */
            0                         /* CPU core to use */
        );
        // esp_task_wdt_config_t config = {
        //     .timeout_ms = 60000,
        //     .idle_core_mask = 0, // Do not watch any idle task
        //     .trigger_panic = true,
        // };
        // esp_task_wdt_reconfigure(&config);
        //ESP_ERROR_CHECK(esp_task_wdt_add(taskHandle_));
    }

    DriverBase::~DriverBase() {
        // Delete step task
        if (taskHandle_ != nullptr)
        {
            vTaskDelete(taskHandle_);
        }
    }

    void IRAM_ATTR DriverBase::task(void* args) {
        DriverBase* self = static_cast<DriverBase*>(args);

        for(;;) {
            Notification notification;
            if (xTaskNotifyWait(0UL, ~0UL, &notification.raw, portMAX_DELAY) != pdTRUE) {
                continue; // Timeout or error
            }

            // Check if direction change is enqueued
            if (notification.data.doDirectionChange) {
                if (notification.data.directionCW) {
                    self->setDirection(Direction::Clockwise);
                }
                else if (notification.data.directionCCW) {
                    self->setDirection(Direction::Counterclockwise);
                }

                uint32_t delay_us = static_cast<uint32_t>(self->directionDelay_us_ + 0.999f);
                esp_rom_delay_us(delay_us); // busy wait, as this should not happen frequently, it should be ok
            }

            // Check if step batch is ready
            if (notification.data.doStep) {
                uint32_t steps = notification.data.doStep;
                self->numStepsDone_ += steps;

                // Execute callback
                self->isrStepThreshold_ = self->callbackOnStepDone_(steps, self->pulsePeriod_us_, self->callbackOnStepDoneUserCtx_);
                self->update(steps, self->pulsePeriod_us_);
            }
        }

        vTaskDelete(self->taskHandle_);
        self->taskHandle_ = nullptr;
    }

    void DriverBase::enable() {
        pinEnable_.enable();
    }

    void DriverBase::disable() {
        pinEnable_.disable();
    }

    bool DriverBase::isEnabled() const {
        return pinEnable_.isEnabled();
    }

    bool DriverBase::doStep(bool immidiately) {
        if (taskHandle_ != nullptr)
        {
            portENTER_CRITICAL(&stepCountMux_);
            isrStepCount_++;
            bool notify = (isrStepCount_ >= isrStepThreshold_);
            portEXIT_CRITICAL(&stepCountMux_);

            if (notify || immidiately) {
                portENTER_CRITICAL(&stepCountMux_);
                uint32_t steps = isrStepCount_;
                isrStepCount_ = 0;
                isrStepThreshold_ = UINT32_MAX; // suppress notifications during callback
                portEXIT_CRITICAL(&stepCountMux_);
                return xTaskNotify(taskHandle_, steps, eNotifyAction::eSetBits);
            }
            return true;
        }
        return false;
    }

    bool IRAM_ATTR DriverBase::doStepFromISR(BaseType_t* pxHigherPriorityTaskWoken, bool immidiately) {
        if (taskHandle_ != nullptr)
        {
            portENTER_CRITICAL(&stepCountMux_);
            isrStepCount_++;
            bool notify = (isrStepCount_ >= isrStepThreshold_);
            portEXIT_CRITICAL(&stepCountMux_);

            if (notify || immidiately) {
                portENTER_CRITICAL(&stepCountMux_);
                uint32_t steps = isrStepCount_;
                isrStepCount_ = 0;
                isrStepThreshold_ = UINT32_MAX; // suppress notifications during callback
                portEXIT_CRITICAL(&stepCountMux_);
                BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
                BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
                return xTaskNotifyFromISR(taskHandle_, steps, eNotifyAction::eSetBits, pFlag);
            }
            return true;
        }
        return false;
    }

    bool DriverBase::setDirectionQueued(Direction direction) {
        Notification notification;
        notification.data.doDirectionChange = 1;

        if (direction == Direction::Clockwise) {
            notification.data.directionCW = 1;
        }
        else if (direction == Direction::Counterclockwise) {
            notification.data.directionCCW = 1;
        }
        else {
            disable();
            return true;
        }

        if (taskHandle_ != nullptr) {
            return xTaskNotify(taskHandle_, notification.raw, eNotifyAction::eSetBits);
        }
        return false;
    }

    bool IRAM_ATTR DriverBase::setDirectionQueuedFromISR(Direction direction, BaseType_t* pxHigherPriorityTaskWoken) {
        Notification notification;
        notification.data.doDirectionChange = 1;

        if (direction == Direction::Clockwise) {
            notification.data.directionCW = 1;
        }
        else if (direction == Direction::Counterclockwise) {
            notification.data.directionCCW = 1;
        }
        else {
            disable();
            return true;
        }

        if (taskHandle_ != nullptr) {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            return xTaskNotifyFromISR(taskHandle_, notification.raw, eNotifyAction::eSetBits, pFlag);
        }
        return false;
    }

    void DriverBase::setDirection(Direction direction) {
        if (direction == Direction::Counterclockwise) {
            pinDirection_.enable();
        }
        else if (direction == Direction::Clockwise) {
            pinDirection_.disable();
        }
        else {
            disable();
        }
    }

    Direction DriverBase::changeDirection()
    {
        if (!isEnabled()) {
            return Direction::Neutral;
        }

        bool newDirection = pinDirection_.toggle();

        if (newDirection) {
            return Direction::Counterclockwise;
        }
        else {
            return Direction::Clockwise;
        }
    }

    Direction DriverBase::getDirection() const
    {
        if (!isEnabled()) {
            return Direction::Neutral;
        }

        if (pinDirection_.isEnabled()) {
            return Direction::Counterclockwise;
        }
        else {
            return Direction::Clockwise;
        }
    }

    void DriverBase::setTiming(float minPulseWidthHigh_us, float minPulseWidthLow_us, float directionDelay_us, float enableDelay_us, float maxPulsePeriod_us)
    {
        minPulseWidthHigh_us_ = minPulseWidthHigh_us;
        minPulseWidthLow_us_ = minPulseWidthLow_us;
        directionDelay_us_ = directionDelay_us;
        enableDelay_us_ = enableDelay_us;
        maxPulsePeriod_us_ = maxPulsePeriod_us;
    }

    float DriverBase::getMinPulsePeriodUs() const
    {
        return minPulseWidthHigh_us_ + minPulseWidthLow_us_;
    }

    float DriverBase::getMaxPulsePeriodUs() const
    {
        return maxPulsePeriod_us_;
    }

    void DriverBase::setPulsePeriodUs(float pulsePeriod_us) {
        pulsePeriod_us_ = pulsePeriod_us;
    }

    float DriverBase::getPulsePeriodUs() const {
        return pulsePeriod_us_;
    }

    uint8_t DriverBase::getMicrosteps() const {
        return microsteps_;
    }

    uint64_t DriverBase::getSteps() const {
        return numStepsDone_;
    }

    void DriverBase::resetSteps(uint64_t count) {
        numStepsDone_ = count;
    }

    uint64_t DriverBase::getStepsMissed() const {
        return numStepsMissed_;
    }

    void DriverBase::resetStepsMissed(uint64_t count) {
        numStepsMissed_ = count;
    }

    void DriverBase::registerCallbackOnStepDone(DriverCallback callback, void* user_ctx) {
        callbackOnStepDoneUserCtx_ = user_ctx;
        callbackOnStepDone_ = std::move(callback);
    }

    void DriverBase::forceStepCallback() {
        isrStepThreshold_ = 1; // next ISR step will trigger a callback
    }
}