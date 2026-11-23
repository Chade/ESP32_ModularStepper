#ifndef STEPPER_DRIVER_ESP32_HPP
#define STEPPER_DRIVER_ESP32_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../Generic/Driver.hpp"
#include "../Generic/Helper.hpp"

namespace Stepper {
namespace ESP32 {
    class Driver : public Generic::Driver {
    public:
        Driver() = delete;
        Driver(Generic::Pin& enablePin, Generic::Pin& stepPin, Generic::Pin& directionPin, uint8_t microsteps = 1);

        ~Driver() override;

        void init() override = 0;
        void start() override = 0;
        void stop() override = 0;
        bool isRunning() override = 0;
        void forceStepCallback() override;
        void reset(bool resetSteps = false) override;
        
        bool setDirectionQueued(Direction direction);
        bool IRAM_ATTR setDirectionQueuedFromISR(Direction direction, BaseType_t* pxHigherPriorityTaskWoken = nullptr);

        uint64_t getStepsFast() const;
        
    protected:
        static void task(void *args);

        // Update the params for the next step cycle
        void update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew, float pulsePeriodIncrement) override = 0;

        bool notifyStepDone(bool immidiately = false);
        bool IRAM_ATTR notifyStepDoneFromISR(BaseType_t* pxHigherPriorityTaskWoken, bool immidiately = false);

        volatile uint32_t isrStepCount_ {0};
        volatile uint32_t isrStepThreshold_ {0};
        portMUX_TYPE stepCountMux_ = portMUX_INITIALIZER_UNLOCKED;

        TaskHandle_t taskHandle_ {nullptr};
    };
} // namespace ESP32
} // namespace Stepper

#endif //STEPPER_DRIVER_ESP32_HPP