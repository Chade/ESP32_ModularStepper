#include "Driver.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {
    Driver::Driver(Generic::Pin* enablePin, Generic::Pin& stepPin, Generic::Pin& directionPin, uint8_t microsteps)
            : Generic::Driver(enablePin, stepPin, directionPin, microsteps) {
    }

    Driver::Driver(Generic::Pin& enablePin, Generic::Pin& stepPin, Generic::Pin& directionPin, uint8_t microsteps)
            : Generic::Driver(enablePin, stepPin, directionPin, microsteps) {
    }

    bool Driver::init() {
        if (taskHandle_ != nullptr) {
            return true;
        }

        // Create step task
        BaseType_t res = xTaskCreatePinnedToCore(
            Driver::task, /* Task function */
            "StepTask",         /* Task name (max. 16 characters by default) */
            8192,               /* Stack size in bytes */
            this,               /* Parameter passed as input of the task */
            10,                 /* Task priority */
            &taskHandle_,       /* Task handle */
            0                   /* CPU core to use */
        );

        if (res != pdPASS) {
            LOG_ERROR("Failed to create FreeRTOS driver task");
            taskHandle_ = nullptr;
            return false;
        }

        return true;
    }

    Driver::~Driver() {
        // Atomically detach taskHandle_ before deleting the task so no ISR can notify a deleted handle
        TaskHandle_t handleToDelete = nullptr;
        portENTER_CRITICAL(&stepCountMux_);
        handleToDelete = taskHandle_;
        taskHandle_ = nullptr;
        portEXIT_CRITICAL(&stepCountMux_);

        if (handleToDelete != nullptr) {
            vTaskDelete(handleToDelete);
        }
    }

    void Driver::task(void* args) {
        Driver* self = static_cast<Driver*>(args);

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
                uint32_t stepsDone = notification.data.doStep;
                self->numStepsDone_ += stepsDone;

                // Execute callback
                uint32_t stepsToDo = 0;
                float pulsePeriodIncrement_us = 0.0f;
                self->callbackOnStepDone_(stepsDone, stepsToDo, self->pulsePeriod_us_, pulsePeriodIncrement_us, self->callbackOnStepDoneUserCtx_);
                if (stepsToDo > 0) {
                    self->isrStepThreshold_ = stepsToDo;
                    self->update(stepsDone, stepsToDo, self->pulsePeriod_us_, pulsePeriodIncrement_us);
                }
                else {
                    self->forceStepCallback();
                    self->stop();
                }
            }
        }
    }

    bool Driver::notifyStepDone(bool immidiately) {
        TaskHandle_t handle = nullptr;
        uint32_t steps = 0;
        bool shouldNotify = false;

        portENTER_CRITICAL(&stepCountMux_);
        handle = taskHandle_;
        if (handle != nullptr) {
            isrStepCount_++;
            if (immidiately || isrStepCount_ >= isrStepThreshold_) {
                steps = isrStepCount_;
                isrStepCount_ = 0;
                isrStepThreshold_ = UINT32_MAX; // suppress notifications during callback
                shouldNotify = true;
            }
        }
        portEXIT_CRITICAL(&stepCountMux_);

        if (handle == nullptr) {
            return false;
        }
        if (shouldNotify) {
            return xTaskNotify(handle, steps, eNotifyAction::eSetBits) == pdPASS;
        }
        return true;
    }

    bool IRAM_ATTR Driver::notifyStepDoneFromISR(BaseType_t* pxHigherPriorityTaskWoken, bool immidiately) {
        TaskHandle_t handle = nullptr;
        uint32_t steps = 0;
        bool shouldNotify = false;

        portENTER_CRITICAL_ISR(&stepCountMux_);
        handle = taskHandle_;
        if (handle != nullptr) {
            isrStepCount_++;
            if (immidiately || isrStepCount_ >= isrStepThreshold_) {
                steps = isrStepCount_;
                isrStepCount_ = 0;
                isrStepThreshold_ = UINT32_MAX; // suppress notifications during callback
                shouldNotify = true;
            }
        }
        portEXIT_CRITICAL_ISR(&stepCountMux_);

        if (handle == nullptr) {
            return false;
        }
        if (shouldNotify) {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            return xTaskNotifyFromISR(handle, steps, eNotifyAction::eSetBits, pFlag) == pdPASS;
        }
        return true;
    }

    void Driver::forceStepCallback() {
        isrStepThreshold_ = 0; // next ISR step will trigger a callback
    }

    void Driver::reset(bool resetSteps) {
        Generic::Driver::reset(resetSteps);
        portENTER_CRITICAL(&stepCountMux_);
        isrStepCount_ = 0;
        isrStepThreshold_ = 0;
        portEXIT_CRITICAL(&stepCountMux_);
    }

    bool Driver::setDirectionQueued(Direction direction) {
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

        portENTER_CRITICAL(&stepCountMux_);
        TaskHandle_t handle = taskHandle_;
        portEXIT_CRITICAL(&stepCountMux_);

        if (handle != nullptr) {
            return xTaskNotify(handle, notification.raw, eNotifyAction::eSetBits) == pdPASS;
        }
        return false;
    }

    bool IRAM_ATTR Driver::setDirectionQueuedFromISR(Direction direction, BaseType_t* pxHigherPriorityTaskWoken) {
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

        portENTER_CRITICAL_ISR(&stepCountMux_);
        TaskHandle_t handle = taskHandle_;
        portEXIT_CRITICAL_ISR(&stepCountMux_);

        if (handle != nullptr) {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            return xTaskNotifyFromISR(handle, notification.raw, eNotifyAction::eSetBits, pFlag) == pdPASS;
        }
        return false;
    }

    uint64_t Driver::getStepsFast() const {
        return numStepsDone_ + isrStepCount_;
    }
} // namespace ESP32
} // namespace Stepper