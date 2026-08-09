#ifndef STEPPER_DRIVER_BASE_HPP
#define STEPPER_DRIVER_BASE_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

#include "StepperPin.hpp"
#include "StepperHelper.hpp"

namespace Stepper {
    using DriverCallback = std::function<void(uint32_t stepsDone, uint32_t& stepsToDo, float& pulsePeriod_us, float& pulsePeriodIncrement_us, void* user_ctx)>;

    class DriverBase {
    public:
        DriverBase() = delete;
        DriverBase(int8_t enablePin, int8_t stepPin, int8_t directionPin, uint8_t microsteps = 1);

        virtual ~DriverBase();

        virtual void init() = 0;
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual bool isRunning() = 0;

        virtual void enable();
        virtual void disable();
        virtual bool isEnabled() const;
        
        virtual void setDirection(Direction direction);
        virtual Direction getDirection() const;
        Direction changeDirection();
        
        bool setDirectionQueued(Direction direction);
        bool setDirectionQueuedFromISR(Direction direction, BaseType_t* pxHigherPriorityTaskWoken = nullptr);
        
        void setTiming(float minPulseWidthHigh_us, float minPulseWidthLow_us, float directionDelay_us, float enableDelay_us);
        float getMinPulsePeriodUs() const;
        float getMaxPulsePeriodUs() const;
        bool setPulsePeriodUs(float pulsePeriod_us);
        float getPulsePeriodUs() const;
        
        bool checkPulsePeriod(float pulsePeriod_us);
        bool checkPulsePeriod(float pulsePeriod_us, float& pulsePeriodNew_us);
        
        uint8_t getMicrosteps() const;
        
        uint64_t getSteps() const;
        uint64_t getStepsFast() const;
        void resetSteps(uint64_t count = 0);
        void reset(bool resetSteps = false);
        
        // Register a callback to calculate the next pulse period, based on current pulse period and the steps done since last call.
        // The callback should return the number of steps for the next batch, and update pulsePeriod_us to the new value to apply for the next batch.
        void registerCallbackOnStepDone(DriverCallback callback, void* user_ctx);
        void forceStepCallback();
        
    protected:
        static void task(void *args);

        // Update the params for the next step cycle
        virtual void update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew, float pulsePeriodIncrement) = 0;
        
        // Notify task if step is done
        bool notifyStepDone(bool immidiately = false);
        bool notifyStepDoneFromISR(BaseType_t* pxHigherPriorityTaskWoken = nullptr, bool immidiately = false);
        
        Pin pinEnable_;
        Pin pinStep_;
        Pin pinDirection_;

        float minPulseWidthHigh_us_ {1.0f};
        float minPulseWidthLow_us_ {1.0f};
        float directionDelay_us_ {0.2f};
        float enableDelay_us_ {0.2f};
        float minPulsePeriod_us_ {1.0f};
        float maxPulsePeriod_us_ {1000000.0f};
        float pulsePeriod_us_ {0.0f};

        uint8_t microsteps_ {1};

        uint64_t numStepsDone_ {0};
        uint64_t numStepsMissed_ {0};

        volatile uint32_t isrStepCount_ {0};
        volatile uint32_t isrStepThreshold_ {0};
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

        TaskHandle_t taskHandle_ {nullptr};

        void* callbackOnStepDoneUserCtx_ {nullptr};
        DriverCallback callbackOnStepDone_ = [this](uint32_t, uint32_t&, float&, float&, void*) {};
        static constexpr const char* log_tag = "Driver";
    };
}

#endif //STEPPER_DRIVER_BASE_HPP