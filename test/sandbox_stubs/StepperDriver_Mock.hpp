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

        Driver(Pin& enablePin, Pin& stepPin, Pin& directionPin, uint8_t microsteps = 1)
                : Stepper::Generic::Driver(enablePin, stepPin, directionPin, microsteps) {};

        virtual void init() {};
        virtual void start() { running_ = true; };
        virtual void stop() { running_ = false; };
        virtual bool isRunning() { return running_; };
        virtual void forceStepCallback() {};
        
    protected:
        void update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew, float pulsePeriodIncrement) {};

        bool running_ {false};
    };
} // namespace Mock
} // namespace Stepper

#endif //STEPPER_DRIVER_MOCK_HPP