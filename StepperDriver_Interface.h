#ifndef STEPPER_DRIVER_INTERFACE_H
#define STEPPER_DRIVER_INTERFACE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

#include "StepperPin.h"
#include "StepperHelper.h"

namespace Stepper {
    using DriverCallback = std::function<bool(uint64_t& stepCount, uint32_t& pulsePeriod_ns)>;

    class DriverInterface {
    public:
        DriverInterface() = delete;
        DriverInterface(int8_t enablePin, int8_t stepPin, int8_t directionPin);

        virtual ~DriverInterface();

        virtual void init() = 0;
        virtual void start() = 0;
        virtual void stop() = 0;

        virtual void enable();
        virtual void disable();
        virtual bool isEnabled() const;

        virtual bool doStep();
        virtual bool doStepFromISR(BaseType_t* pxHigherPriorityTaskWoken = nullptr);

        virtual bool setDirectionQueued(Direction direction);
        virtual bool setDirectionQueuedFromISR(Direction direction, BaseType_t* pxHigherPriorityTaskWoken = nullptr);
        virtual void setDirection(Direction direction);
        virtual Direction changeDirection();
        virtual Direction getDirection() const;

        virtual void setTiming(uint32_t stepPulseWidthHigh_ns, uint32_t stepPulseWidthLow_ns, uint32_t directionDelay_ns, uint32_t enableDelay_ns, uint32_t maxPulsePeriod_ns);
        virtual uint32_t getMinPulsePeriodNs() const;
        virtual uint32_t getMaxPulsePeriodNs() const;

        virtual void setMicrosteps(uint8_t microsteps);
        virtual uint8_t getMicrosteps() const;

        virtual int64_t getSteps() const;
        virtual void resetSteps(uint64_t count = 0);

        virtual int64_t getStepsMissed() const;
        virtual void resetStepsMissed(uint64_t count = 0);

        void registerCallbackOnStepDone(DriverCallback callback);

    protected:
        static void task(void *args);
        virtual bool taskLoop(uint32_t notificationValue) = 0;

        Pin pinEnable_;
        Pin pinStep_;
        Pin pinDirection_;

        uint32_t stepPulseWidthHigh_ns_ {1000};
        uint32_t stepPulseWidthLow_ns_ {1000};
        uint32_t directionDelay_ns_ {1000};
        uint32_t enableDelay_ns_ {1000};
        uint32_t maxPulsePeriod_ns_ {1000000};

        uint8_t microsteps_ {1};

        uint64_t numStepsDone_ {0};
        uint64_t numStepsMissed_ {0};

        static constexpr uint32_t ulDirectionBitmaskCW_ = 1UL << 30;
        static constexpr uint32_t ulDirectionBitmaskCCW_ = 2UL << 30;
        static constexpr uint32_t ulDoDirectionChangeBitmask_ = 3UL << 30;
        static constexpr uint32_t ulDoStepBitmask_ = ~ulDoDirectionChangeBitmask_;

        TaskHandle_t taskHandle_ {nullptr};

        DriverCallback callbackOnStepDone_ = [this](uint64_t&, uint32_t&) { return true; };
    };
}

#endif //STEPPER_DRIVER_INTERFACE_H