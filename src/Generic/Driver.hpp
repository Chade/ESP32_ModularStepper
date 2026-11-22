#ifndef STEPPER_DRIVER_HPP
#define STEPPER_DRIVER_HPP

#include <functional>

#include "Pin.hpp"
#include "Helper.hpp"

namespace Stepper {
namespace Generic {
    using DriverCallback = std::function<void(uint32_t stepsDone, uint32_t& stepsToDo, float& pulsePeriod_us, float& pulsePeriodIncrement_us, void* user_ctx)>;

    class Driver {
    public:
        Driver() = delete;
        Driver(Generic::Pin& enablePin, Generic::Pin& stepPin, Generic::Pin& directionPin, uint8_t microsteps = 1);

        virtual ~Driver() = default;

        virtual void init() = 0;
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual bool isRunning() = 0;
        virtual void forceStepCallback() = 0;

        virtual void enable();
        virtual void disable();
        virtual bool isEnabled() const;
        
        virtual void setDirection(Direction direction);
        virtual Direction getDirection() const;
        Direction changeDirection();

        void setTiming(float minPulseWidthHigh_us, float minPulseWidthLow_us, float directionDelay_us, float enableDelay_us);
        float getMinPulsePeriodUs() const;
        float getMaxPulsePeriodUs() const;
        bool setPulsePeriodUs(float pulsePeriod_us);
        float getPulsePeriodUs() const;
        
        bool checkPulsePeriod(float pulsePeriod_us);
        bool checkPulsePeriod(float pulsePeriod_us, float& pulsePeriodNew_us);
        
        uint8_t getMicrosteps() const;
        uint64_t getSteps() const;

        void resetSteps(uint64_t count = 0);
        virtual void reset(bool resetSteps = false);
        
        // Register a callback to calculate the next pulse period, based on current pulse period and the steps done since last call.
        // The callback should return the number of steps for the next batch, and update pulsePeriod_us to the new value to apply for the next batch.
        void registerCallbackOnStepDone(DriverCallback callback, void* user_ctx);
        
    protected:
        // Update the params for the next step cycle
        virtual void update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew, float pulsePeriodIncrement) = 0;
        
        Generic::Pin& pinEnable_;
        Generic::Pin& pinStep_;
        Generic::Pin& pinDirection_;

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

        void* callbackOnStepDoneUserCtx_ {nullptr};
        DriverCallback callbackOnStepDone_ = [this](uint32_t, uint32_t&, float&, float&, void*) {};
    };
} // namespace Generic
} // namespace Stepper

#endif //STEPPER_DRIVER_HPP