#ifndef STEPPER_DRIVER_INTERFACE_H
#define STEPPER_DRIVER_INTERFACE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

#include "StepperPin.h"
#include "StepperHelper.h"

namespace Stepper {
    using DriverCallback = std::function<uint32_t (uint32_t stepsNew, uint32_t pulsePeriod_ns, void* user_ctx)>;

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

        virtual void setTiming(uint32_t minPulseWidthHigh_ns, uint32_t minPulseWidthLow_ns, uint32_t directionDelay_ns, uint32_t enableDelay_ns, uint32_t maxPulsePeriod_ns);
        virtual uint32_t getMinPulsePeriodNs() const;
        virtual uint32_t getMaxPulsePeriodNs() const;
        virtual void setPulsePeriodNs(uint32_t pulsePeriod_ns);
        virtual uint32_t getPulsePeriod() const;

        virtual void setMicrosteps(uint8_t microsteps);
        virtual uint8_t getMicrosteps() const;

        virtual int64_t getSteps() const;
        virtual void resetSteps(uint64_t count = 0);

        virtual int64_t getStepsMissed() const;
        virtual void resetStepsMissed(uint64_t count = 0);

        void registerCallbackOnStepDone(DriverCallback callback, void* user_ctx);

    protected:
        static void task(void *args);
        virtual void update(uint32_t stepsNew, uint32_t pulsePeriodNew) = 0;

        Pin pinEnable_;
        Pin pinStep_;
        Pin pinDirection_;

        uint32_t minPulseWidthHigh_ns_ {1000};
        uint32_t minPulseWidthLow_ns_ {1000};
        uint32_t directionDelay_ns_ {200};
        uint32_t enableDelay_ns_ {200};
        uint32_t maxPulsePeriod_ns_ {13107000};
        uint32_t pulsePeriod_ns_ {0};

        uint8_t microsteps_ {1};

        uint64_t numStepsDone_ {0};
        uint64_t numStepsMissed_ {0};

        struct StepTask {
            uint32_t steps;
            uint32_t pulsePeriodTicks;
        };

        struct NotificationData {
            uint32_t doStep : 29;
            uint8_t doDirectionChange : 1;
            uint8_t directionCW :1;
            uint8_t directionCCW : 1;
        };

        union Notification {
            NotificationData data;
            uint32_t raw;
        };

        static constexpr uint32_t ulDirectionBitmaskCW_ = 1UL << 30;
        static constexpr uint32_t ulDirectionBitmaskCCW_ = 2UL << 30;
        static constexpr uint32_t ulDoDirectionChangeBitmask_ = 3UL << 30;
        static constexpr uint32_t ulDoStepBitmask_ = ~ulDoDirectionChangeBitmask_;

        TaskHandle_t taskHandle_ {nullptr};

        void* callbackOnStepDoneUserCtx_ {nullptr};
        DriverCallback callbackOnStepDone_ = [this](uint32_t, uint32_t, void*) -> uint32_t { return pulsePeriod_ns_; };
    };
}

#endif //STEPPER_DRIVER_INTERFACE_H