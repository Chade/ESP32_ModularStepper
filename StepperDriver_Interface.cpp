#include "esp32-hal-gpio.h"
#include "StepperDriver_Interface.h"
#include "esp_task_wdt.h"

namespace Stepper {
    DriverInterface::DriverInterface(int8_t enablePin, int8_t stepPin, int8_t directionPin)
            : pinEnable_(enablePin), pinStep_(stepPin), pinDirection_(directionPin) {
        
        // Create step task
        xTaskCreatePinnedToCore(
            DriverInterface::task,    /* Task function */
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

    DriverInterface::~DriverInterface() {
        // Delete step task
        if (taskHandle_ != nullptr)
        {
            vTaskDelete(taskHandle_);
        }
    }

    void IRAM_ATTR DriverInterface::task(void* args) {
        DriverInterface* self = static_cast<DriverInterface*>(args);

        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)13, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)13, 0));

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

                uint32_t delay_us = static_cast<uint32_t>((self->directionDelay_ns_ + 999) / 1000); // ceil(ns/1000)
                esp_rom_delay_us(delay_us); // busy wait, as this should not happen frequently, it should be ok
            }

            // Check if step is enqueued
            if (notification.data.doStep) {;
                self->numStepsDone_ += notification.data.doStep;
                self->numStepsMissed_ += notification.data.doStep - 1;

                // Execute callback
                ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)13, 1));
                uint32_t newPulsePeriod_ns = self->callbackOnStepDone_(notification.data.doStep, self->pulsePeriod_ns_, self->callbackOnStepDoneUserCtx_);
                ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)13, 0));
                self->setPulsePeriodNs(newPulsePeriod_ns);
                self->update(notification.data.doStep, newPulsePeriod_ns);
            }
            taskYIELD();
        }

        vTaskDelete(self->taskHandle_);
        self->taskHandle_ = nullptr;
    }

    void DriverInterface::enable() {
        pinEnable_.enable();
    }

    void DriverInterface::disable() {
        pinEnable_.disable();
    }

    bool DriverInterface::isEnabled() const {
        return pinEnable_.isEnabled();
    }

    bool DriverInterface::doStep() {
        if (taskHandle_ != nullptr)
        {
            return xTaskNotify(taskHandle_, 0, eNotifyAction::eIncrement);
        }
        return false;
    }

    bool IRAM_ATTR DriverInterface::doStepFromISR(BaseType_t* pxHigherPriorityTaskWoken) {
        if (taskHandle_ != nullptr)
        {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            return xTaskNotifyFromISR(taskHandle_, 0, eNotifyAction::eIncrement, pFlag);
        }
        return false;
    }

    bool DriverInterface::setDirectionQueued(Direction direction) {
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

    bool IRAM_ATTR DriverInterface::setDirectionQueuedFromISR(Direction direction, BaseType_t* pxHigherPriorityTaskWoken) {
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

    void DriverInterface::setDirection(Direction direction) {
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

    Direction DriverInterface::changeDirection()
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

    Direction DriverInterface::getDirection() const
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

    void DriverInterface::setTiming(uint32_t minPulseWidthHigh_ns, uint32_t minPulseWidthLow_ns, uint32_t directionDelay_ns, uint32_t enableDelay_ns, uint32_t maxPulsePeriod_ns)
    {
        minPulseWidthHigh_ns_ = minPulseWidthHigh_ns;
        minPulseWidthLow_ns_ = minPulseWidthLow_ns;
        directionDelay_ns_ = directionDelay_ns;
        enableDelay_ns_ = enableDelay_ns;
        maxPulsePeriod_ns_ = maxPulsePeriod_ns;
    }

    uint32_t DriverInterface::getMinPulsePeriodNs() const
    {
        return minPulseWidthHigh_ns_ + minPulseWidthLow_ns_;
    }

    uint32_t DriverInterface::getMaxPulsePeriodNs() const
    {
        return maxPulsePeriod_ns_;
    }

    void DriverInterface::setPulsePeriodNs(uint32_t pulsePeriod_ns) {
        pulsePeriod_ns_ = pulsePeriod_ns;
    }

    uint32_t DriverInterface::getPulsePeriod() const {
        return pulsePeriod_ns_;
    }

    void DriverInterface::setMicrosteps(uint8_t microsteps) {
        microsteps_ = microsteps;
    }

    uint8_t DriverInterface::getMicrosteps() const {
        return microsteps_;
    }

    int64_t DriverInterface::getSteps() const {
        return numStepsDone_;
    }

    void DriverInterface::resetSteps(uint64_t count) {
        numStepsDone_ = count;
    }

    int64_t DriverInterface::getStepsMissed() const {
        return numStepsMissed_;
    }

    void DriverInterface::resetStepsMissed(uint64_t count) {
        numStepsMissed_ = count;
    }

    void DriverInterface::registerCallbackOnStepDone(DriverCallback callback, void* user_ctx) {
        callbackOnStepDoneUserCtx_ = user_ctx;
        callbackOnStepDone_ = std::move(callback);
    }
}