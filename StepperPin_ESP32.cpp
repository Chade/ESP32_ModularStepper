#include "StepperPin_ESP32.hpp"

namespace Stepper {
namespace ESP32 {

    Pin::Pin(int8_t pin,Mode mode)
            : Stepper::Generic::Pin(pin, mode) {
        setup(mode);
    }

    Pin::Pin(uint8_t pin, bool inverse,Mode mode)
            : Stepper::Generic::Pin(pin, inverse, mode) {
        setup(mode);
    }

    void Pin::setup(Mode mode) {
        gpio_mode_t gpioMode;
        switch(mode)
        {
            case Mode::Disable:
                gpioMode = GPIO_MODE_DISABLE;
                break;
            case Mode::Input:
                gpioMode = GPIO_MODE_INPUT;
                break;
            case Mode::Output:
                gpioMode = GPIO_MODE_OUTPUT;
                break;
            case Mode::Input_Output:
                gpioMode = GPIO_MODE_INPUT_OUTPUT;
                break;
        }
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)gpio_, gpioMode));
    }

    bool Pin::isEnabled() const{
        uint32_t level = gpio_get_level((gpio_num_t)gpio_);
        return (level == enableLevel_);
    }

    bool Pin::isDisabled() const {
        uint32_t level = gpio_get_level((gpio_num_t)gpio_);
        return (level != enableLevel_);
    }

    void Pin::enable() {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, enableLevel_));
    }

    void Pin::disable() {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, !enableLevel_));
    }

    void Pin::setLevel(bool level) {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, level));
    }

    bool Pin::getLevel() const{
        uint32_t level = gpio_get_level((gpio_num_t)gpio_);
        return level;
    }
} // namespace ESP32
} // namespace Stepper
