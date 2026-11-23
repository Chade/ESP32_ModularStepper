#ifndef STEPPER_PIN_ESP32_HPP
#define STEPPER_PIN_ESP32_HPP

#include <driver/gpio.h>

#include "../Generic/Pin.hpp"

namespace Stepper {
namespace ESP32 {
    class Pin : public Generic::Pin {
    public:
        Pin() = delete;
        Pin(int8_t pin, IoMode mode = IoMode::Input_Output, PullMode pullMode = PullMode::Floating);
        Pin(uint8_t pin, bool inverse, IoMode mode = IoMode::Input_Output, PullMode pullMode = PullMode::Floating);

        void setIoMode(IoMode mode) override;
        void setPullMode(PullMode pullMode) override;
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