#ifndef STEPPER_PIN_H
#define STEPPER_PIN_H

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
            disableLevel_  = (pin > 0) ? 0 : 1;

            setup(mode);
        }

        Pin(uint8_t pin, bool inverse = false, Mode mode = Mode::Input_Output) {
            gpio_ = pin;
            disableLevel_ = inverse;
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
            return (level > 0) ? enableLevel_ : disableLevel_;
        };

        void enable() {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, enableLevel_));
        };

        void disable() {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, disableLevel_));
        };

        void setLevel(bool level) {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio_, level));
        }

        bool getLevel() const {
            uint32_t level = gpio_get_level((gpio_num_t)gpio_);
            return level;
        }

        bool getLevelEnable() const {
            return enableLevel_;
        }

        bool getLevelDisable() const {
            return disableLevel_;
        }
        
        bool toggle() {
            bool level = !getLevel();
            setLevel(level);
            return level;
        }


    private:
        uint8_t gpio_ {0};
        bool enableLevel_ {1};
        bool disableLevel_ {0};
    };

    

}



#endif // STEPPER_PIN_H