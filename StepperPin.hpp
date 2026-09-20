#ifndef STEPPER_PIN_HPP
#define STEPPER_PIN_HPP

#include <cstdint>

namespace Stepper {
namespace Generic {
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
        }

        Pin(uint8_t pin, bool inverse, Mode mode = Mode::Input_Output) {
            gpio_ = pin;
            enableLevel_ = !inverse;
        };

        virtual ~Pin() = default;

        virtual void setup(Mode mode) = 0;
        virtual bool isEnabled() const = 0;
        virtual bool isDisabled() const = 0;
        virtual void enable() = 0;
        virtual void disable() = 0;
        virtual void setLevel(bool level) = 0;
        virtual bool getLevel() const = 0;
        
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
        
        uint8_t getPin() const {
            return gpio_;
        }
        
    protected:
        uint8_t gpio_ {0};
        bool enableLevel_ {1};
    };
} // namespace Generic
} // namespace Stepper


#endif // STEPPER_PIN_HPP