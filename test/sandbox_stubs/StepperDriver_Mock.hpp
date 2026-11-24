#ifndef STEPPER_DRIVER_MOCK_HPP
#define STEPPER_DRIVER_MOCK_HPP

#include <functional>

#include "Generic/Driver.hpp"
#include "StepperPin_Mock.hpp"

namespace Stepper {
namespace Mock {

    class Driver : public Stepper::Generic::Driver {
    public:
        Driver() = delete;

        Driver(Pin* enablePin, Pin& stepPin, Pin& directionPin, uint8_t microsteps = 1)
                : Stepper::Generic::Driver(enablePin, stepPin, directionPin, microsteps) {};

        Driver(Pin& enablePin, Pin& stepPin, Pin& directionPin, uint8_t microsteps = 1)
                : Stepper::Generic::Driver(enablePin, stepPin, directionPin, microsteps) {};

        bool init() override {
            initCalls_++;
            initialized_ = initShouldSucceed_;
            return initShouldSucceed_;
        }

        void start() override {
            running_ = true;
            nextBatch_ = 1;
        }

        void stop() override {
            running_ = false;
        }

        bool isRunning() override {
            return running_;
        }

        void forceStepCallback() override {
            nextBatch_ = 1;
        }

        bool stepSingleBatch() {
            if (!running_) {
                return false;
            }
            uint32_t done = nextBatch_;
            uint32_t next = 0;
            float inc = 0.0f;
            callbackOnStepDone_(done, next, pulsePeriod_us_, inc, callbackOnStepDoneUserCtx_);
            if (next == 0) {
                stop();
                return false;
            }
            nextBatch_ = next;
            return true;
        }

        bool stepLoop(uint32_t maxCallbacks = 100000) {
            uint32_t count = 0;
            while (running_ && count < maxCallbacks) {
                if (!stepSingleBatch()) {
                    break;
                }
                count++;
            }
            return !running_;
        }

        int initCalls_ {0};
        bool initShouldSucceed_ {true};
        bool initialized_ {false};

    protected:
        void update(uint32_t /*stepsDone*/, uint32_t /*stepsToDo*/, float /*pulsePeriodNew*/, float /*pulsePeriodIncrement*/) override {};

        bool running_ {false};
        uint32_t nextBatch_ {1};
    };
} // namespace Mock
} // namespace Stepper

#endif //STEPPER_DRIVER_MOCK_HPP
