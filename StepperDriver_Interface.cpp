#include "StepperDriver_Interface.h"

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
    }

    DriverInterface::~DriverInterface() {
        // Delete step task
        if (taskHandle_ != nullptr)
        {
            vTaskDelete(taskHandle_);
        }
    }

    void DriverInterface::task(void* args) {
        DriverInterface* self = static_cast<DriverInterface*>(args);

        bool stop = false;
        while (!stop) {
            uint32_t ulNotifiedValue;
            if (xTaskNotifyWait(0, ulDoDirectionChangeBitmask_ | ulDoStepBitmask_, &ulNotifiedValue, portMAX_DELAY) != pdTRUE) {
                continue; // Timeout or error
            }

            // Check if direction change is enqueued
            if (ulNotifiedValue & ulDoDirectionChangeBitmask_) {
                if (ulNotifiedValue & ulDirectionBitmaskCW_) {
                    self->setDirection(Direction::Clockwise);
                }
                else if (ulNotifiedValue & ulDirectionBitmaskCCW_) {
                    self->setDirection(Direction::Counterclockwise);
                }

                uint32_t delay_us = static_cast<uint32_t>((self->directionDelay_ns_ + 999) / 1000); // ceil(ns/1000)
                esp_rom_delay_us(delay_us);
            }

            // Check if step is enqueued
            if (ulNotifiedValue & ulDoStepBitmask_) {
                uint32_t notificationCount = ulNotifiedValue & ulDoStepBitmask_;
                self->numStepsDone_ += 1;
                self->numStepsMissed_ += notificationCount - 1;
                stop = self->taskLoop(notificationCount);
            }
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

    bool DriverInterface::doStep()
    {
        if (taskHandle_ != nullptr)
        {
            return xTaskNotify(taskHandle_, ulDoStepBitmask_, eNotifyAction::eIncrement);
        }
        return false;
    }

    bool IRAM_ATTR DriverInterface::doStepFromISR(BaseType_t* pxHigherPriorityTaskWoken)
    {
        if (taskHandle_ != nullptr)
        {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            return xTaskNotifyFromISR(taskHandle_, ulDoStepBitmask_, eNotifyAction::eIncrement, pFlag);
        }
        return false;
    }

    bool DriverInterface::setDirectionQueued(Direction direction) {
        uint32_t directionBitmask;
        if (direction == Direction::Clockwise) {
            directionBitmask = ulDirectionBitmaskCW_;
        }
        else if (direction == Direction::Counterclockwise) {
            directionBitmask = ulDirectionBitmaskCCW_;
        }
        else {
            disable();
            return true;
        }

        if (taskHandle_ != nullptr) {
            return xTaskNotify(taskHandle_, directionBitmask, eNotifyAction::eSetBits);
        }
        return false;
    }

    bool IRAM_ATTR DriverInterface::setDirectionQueuedFromISR(Direction direction, BaseType_t* pxHigherPriorityTaskWoken)
    {
        uint32_t directionBitmask;
        if (direction == Direction::Clockwise) {
            directionBitmask = ulDirectionBitmaskCW_;
        }
        else if (direction == Direction::Counterclockwise) {
            directionBitmask = ulDirectionBitmaskCCW_;
        }
        else {
            disable();
            return true;
        }

        if (taskHandle_ != nullptr) {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            return xTaskNotifyFromISR(taskHandle_, directionBitmask, eNotifyAction::eSetBits, pFlag);
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

    void DriverInterface::setTiming(uint32_t stepPulseWidthHigh_ns, uint32_t stepPulseWidthLow_ns, uint32_t directionDelay_ns, uint32_t enableDelay_ns, uint32_t maxPulsePeriod_ns)
    {
        stepPulseWidthHigh_ns_ = stepPulseWidthHigh_ns;
        stepPulseWidthLow_ns_ = stepPulseWidthLow_ns;
        directionDelay_ns_ = directionDelay_ns;
        enableDelay_ns_ = enableDelay_ns;
        maxPulsePeriod_ns_ = maxPulsePeriod_ns;
    }

    uint32_t DriverInterface::getMinPulsePeriodNs() const
    {
        return stepPulseWidthHigh_ns_ + stepPulseWidthLow_ns_;
    }

    uint32_t DriverInterface::getMaxPulsePeriodNs() const
    {
        return maxPulsePeriod_ns_;
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

    void DriverInterface::registerCallbackOnStepDone(DriverCallback callback) {
        callbackOnStepDone_ = std::move(callback);
    }
}