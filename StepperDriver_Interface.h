#ifndef STEPPER_DRIVER_INTERFACE_H
#define STEPPER_DRIVER_INTERFACE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

#include "StepperPin.h"
#include "StepperHelper.h"

namespace Stepper {
    using DriverCallback = std::function<uint32_t (uint32_t stepsNew, float& pulsePeriod_us, void* user_ctx)>;

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

        virtual void setTiming(float minPulseWidthHigh_us, float minPulseWidthLow_us, float directionDelay_us, float enableDelay_us, float maxPulsePeriod_us);
        virtual float getMinPulsePeriodUs() const;
        virtual float getMaxPulsePeriodUs() const;
        virtual void setPulsePeriodUs(float pulsePeriod_us);
        virtual float getPulsePeriodUs() const;

        virtual void setMicrosteps(uint8_t microsteps);
        virtual uint8_t getMicrosteps() const;

        virtual uint64_t getSteps() const;
        virtual void resetSteps(uint64_t count = 0);

        virtual uint64_t getStepsMissed() const;
        virtual void resetStepsMissed(uint64_t count = 0);

        void registerCallbackOnStepDone(DriverCallback callback, void* user_ctx);
        void forceStepCallback();

    protected:
        static void task(void *args);
        virtual void update(uint32_t stepsNew, float pulsePeriodNew) = 0;

        Pin pinEnable_;
        Pin pinStep_;
        Pin pinDirection_;

        float minPulseWidthHigh_us_ {1.0f};
        float minPulseWidthLow_us_ {1.0f};
        float directionDelay_us_ {0.2f};
        float enableDelay_us_ {0.2f};
        float maxPulsePeriod_us_ {13107.0f};
        float pulsePeriod_us_ {0.0f};

        uint8_t microsteps_ {1};

        uint64_t numStepsDone_ {0};
        uint64_t numStepsMissed_ {0};

        volatile uint32_t isrStepCount_ {0};
        volatile uint32_t isrStepThreshold_ {1};
        portMUX_TYPE stepCountMux_ = portMUX_INITIALIZER_UNLOCKED;

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
        DriverCallback callbackOnStepDone_ = [this](uint32_t, float& pulsePeriod_us, void*) -> uint32_t { return 1; };
    };
}

#endif //STEPPER_DRIVER_INTERFACE_H