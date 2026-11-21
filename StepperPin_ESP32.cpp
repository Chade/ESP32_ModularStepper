#include "StepperPin_ESP32.hpp"

namespace Stepper {
namespace ESP32 {

    Pin::Pin(int8_t pin,IoMode ioMode, PullMode pullMode)
            : Generic::Pin(pin, ioMode, pullMode) {
        setIoMode(ioMode);
        setPullMode(pullMode);
    }

    Pin::Pin(uint8_t pin, bool inverse,IoMode ioMode, PullMode pullMode)
            : Generic::Pin(pin, inverse, ioMode, pullMode) {
        setIoMode(ioMode);
        setPullMode(pullMode);
    }

    void Pin::setIoMode(IoMode mode) {
        gpio_mode_t gpioMode;
        switch(mode) {
            case IoMode::Disable:
                gpioMode = GPIO_MODE_DISABLE;
                break;
            case IoMode::Input:
                gpioMode = GPIO_MODE_INPUT;
                break;
            case IoMode::Output:
                gpioMode = GPIO_MODE_OUTPUT;
                break;
            case IoMode::Input_Output:
                gpioMode = GPIO_MODE_INPUT_OUTPUT;
                break;
        }
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)gpio_, gpioMode));
    }

    void Pin::setPullMode(PullMode pullMode) {
        gpio_pull_mode_t gpioPullMode;
        switch (pullMode) {
            case PullMode::PullUp:
                gpioPullMode = GPIO_PULLUP_ONLY;
                break;
            case PullMode::PullDown:
                gpioPullMode = GPIO_PULLDOWN_ONLY;
                break;
            case PullMode::Floating:
                gpioPullMode = GPIO_FLOATING;
                break;
        }
        ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)gpio_, gpioPullMode));
    }

    bool Pin::isEnabled() const {
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
