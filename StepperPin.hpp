#ifndef STEPPER_PIN_HPP
#define STEPPER_PIN_HPP

#include <driver/gpio.h>

namespace Stepper {
    class Pin {
    public:
        enum class Mode : uint8_t {
            Disable = 0,
            Input = 1,
            Output = 2,
            Input_Output = 3
        };

        Pin() = delete;

        Pin(int8_t pin, Mode mode = Mode::Input_Output) {
            gpio_ = (pin > 0) ? pin : -pin;
            enableLevel_ = (pin > 0) ? 1 : 0;

            setup(mode);
        }

        Pin(uint8_t pin, bool inverse = false, Mode mode = Mode::Input_Output) {
            gpio_ = pin;
            enableLevel_ = !inverse;

            setup(mode);
        };

        void setup(Mode mode) {
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
        };

        uint8_t getPin() const {
            return gpio_;
        }

        bool isEnabled() const {
            uint32_t level = gpio_get_level((gpio_num_t)gpio_);
            return (level == enableLevel_);
        };

        bool isDisabled() const {
            uint32_t level = gpio_get_level((gpio_num_t)gpio_);
            return (level != enableLevel_);
        };

        void enable() {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, enableLevel_));
        };

        void disable() {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, !enableLevel_));
        };

        void setLevel(bool level) {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, level));
        }

        bool getLevel() const {
            uint32_t level = gpio_get_level((gpio_num_t)gpio_);
            return level;
        }

        bool getLevelEnabled() const {
            return enableLevel_;
        }

        bool getLevelDisabled() const {
            return !enableLevel_;
        }
        
        bool toggle() {
            bool level = !getLevel();
            setLevel(level);
            return level;
        }


    private:
        uint8_t gpio_ {0};
        bool enableLevel_ {1};
    };
}



#endif // STEPPER_PIN_HPP