#include "Driver.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {
    Driver::Driver(Generic::Pin& enablePin, Generic::Pin& stepPin, Generic::Pin& directionPin, uint8_t microsteps)
            : Generic::Driver(enablePin, stepPin, directionPin, microsteps) {

        // Create step task
        xTaskCreatePinnedToCore(
            Driver::driverTask, /* Task function */
            "StepTask",         /* Task name (max. 16 characters by default) */
            8192,               /* Stack size in bytes */
            this,               /* Parameter passed as input of the task */
            10,                 /* Task priority */
            &taskHandle_,       /* Task handle */
            0                   /* CPU core to use */
        );
        // esp_task_wdt_config_t config = {
        //     .timeout_ms = 60000,
        //     .idle_core_mask = 0, // Do not watch any idle task
        //     .trigger_panic = true,
        // };
        // esp_task_wdt_reconfigure(&config);
        //ESP_ERROR_CHECK(esp_task_wdt_add(taskHandle_));
    }

    Driver::~Driver() {
        // Delete step task
        if (taskHandle_ != nullptr)
        {
            vTaskDelete(taskHandle_);
        }
    }

    void Driver::driverTask(void* args) {
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

    bool IRAM_ATTR Driver::notifyStepDoneFromISR(BaseType_t* pxHigherPriorityTaskWoken, bool immidiately) {
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

        if (taskHandle_ != nullptr) {
            return xTaskNotify(taskHandle_, notification.raw, eNotifyAction::eSetBits);
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

        if (taskHandle_ != nullptr) {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t* pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            return xTaskNotifyFromISR(taskHandle_, notification.raw, eNotifyAction::eSetBits, pFlag);
        }
        return false;
    }

    uint64_t Driver::getStepsFast() const {
        return numStepsDone_ + isrStepCount_;
    }
} // namespace ESP32
} // namespace Stepper