#ifndef STEPPER_PIN_ESP32_HPP
#define STEPPER_PIN_ESP32_HPP

#include <driver/gpio.h>
#include "StepperPin.hpp"

namespace Stepper {
namespace ESP32 {
    class Pin : public Stepper::Generic::Pin {
    public:
        Pin() = delete;
        Pin(int8_t pin, Mode mode = Mode::Input_Output);
        Pin(uint8_t pin, bool inverse, Mode mode = Mode::Input_Output);

        void setup(Mode mode) override;
        bool isEnabled() const override;
        bool isDisabled() const override;
        void enable() override;
        void disable() override;
        void setLevel(bool level) override;
        bool getLevel() const override;
    };
} // namespace ESP32
} // namespace Stepper



#endif // STEPPER_PIN_ESP32_HPP