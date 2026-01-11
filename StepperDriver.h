#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include "StepperHelper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <driver/pulse_cnt.h>
#include <functional>

#include <memory>

namespace Stepper
{
    typedef std::function<void(uint64_t currentSteps)> DriverCallback;

    class Driver
    {
    public:
        Driver() = delete;

        Driver(uint8_t enablePin, uint8_t stepPin, uint8_t directionPin, bool inverseDirection = false);

        virtual ~Driver();

        void start();
        void stop();

        void doStep();
        void doStepFromISR(BaseType_t *pxHigherPriorityTaskWoken = nullptr);
        void setDirection(Direction direction);
        void setDirectionFromISR(Direction direction, BaseType_t *pxHigherPriorityTaskWoken = nullptr);
        Direction getDirection();
        Direction changeDirection();
        void registerCallback(DriverCallback callback);
        void setTimings(uint32_t stepPulseWidthHigh_us, uint32_t stepPulseWidthLow_us, uint32_t directionDelay_ns, uint32_t maxPulsePeriodUs = 3200);
        uint32_t getMinPulsePeriodUs();
        uint32_t getMaxPulsePeriodUs();
        void enable();
        void disable();
        bool isEnabled();
        int32_t getCount();
        void resetCount();

    protected:
        const uint8_t m_pinEnable{0};
        const uint8_t m_pinStep{0};
        const uint8_t m_pinDirection{0};

        const bool m_inverseDirection{false};

        uint32_t m_stepPulseWidthHigh_us{1};
        uint32_t m_stepPulseWidthLow_us{1};
        uint32_t m_directionDelay_ns{200};
        uint32_t m_maxPulsePeriodUs{3200};

        DriverCallback m_callback = [this](uint64_t) {};

    private:
        static void stepTask(void *args);

        static constexpr uint32_t ulDoStepBitmask = 1UL << 0;
        static constexpr uint32_t ulDoDirectionChangeBitmask = 1UL << 1;
        static constexpr uint32_t ulDirectionBitmask = 1UL << 2;

        TaskHandle_t m_taskHandle{nullptr};
        pcnt_unit_handle_t m_pcntUnitHandle{nullptr};
        pcnt_channel_handle_t m_pctChannelHandle{nullptr};
    };

}

#endif // STEPPER_DRIVER_H