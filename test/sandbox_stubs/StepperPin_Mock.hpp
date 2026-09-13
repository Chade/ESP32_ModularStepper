#ifndef STEPPER_PIN_MOCK_HPP
#define STEPPER_PIN_MOCK_HPP

#include "StepperPin.hpp"

namespace Stepper {
namespace Mock {
    class Pin : public Stepper::Generic::Pin {
    public:
        Pin()
                : Stepper::Generic::Pin(0) {};

        Pin(int8_t pin, Mode mode = Mode::Input_Output) 
                : Stepper::Generic::Pin(pin, mode) {};

        Pin(uint8_t pin, bool inverse,Mode mode = Mode::Input_Output)
                : Stepper::Generic::Pin(pin, inverse, mode) {};

        void setup(Mode mode) {};
        bool isEnabled() const { return true; };
        bool isDisabled() const { return false; };
        void enable() {};
        void disable() {};
        void setLevel(bool level) {};
        bool getLevel() const { return true; };
    };
} // namespace Mock
} // namespace Stepper


#endif // STEPPER_PIN_MOCK_HPP