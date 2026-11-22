#ifndef STEPPER_PIN_HPP
#define STEPPER_PIN_HPP

#include <cstdint>

namespace Stepper {
namespace Generic {
    class Pin {
    public:
        enum class IoMode : uint8_t {
            Disable = 0,
            Input = 1,
            Output = 2,
            Input_Output = 3
        };

        enum class PullMode : uint8_t {
            Floating = 0,
            PullUp = 1,
            PullDown = 2
        };

        Pin() = delete;

        Pin(int8_t pin, IoMode ioMode = IoMode::Input_Output, PullMode pullMode = PullMode::Floating) {
            gpio_ = (pin > 0) ? pin : -pin;
            enableLevel_ = (pin > 0) ? 1 : 0;
            ioMode_ = ioMode;
            pullMode_ = pullMode;
        }

        Pin(uint8_t pin, bool inverse, IoMode ioMode = IoMode::Input_Output, PullMode pullMode = PullMode::Floating) {
            gpio_ = pin;
            enableLevel_ = !inverse;
            ioMode_ = ioMode;
            pullMode_ = pullMode;
        };

        virtual ~Pin() = default;

        virtual void setIoMode(IoMode mode) = 0;
        virtual void setPullMode(PullMode pullMode) = 0;
        virtual bool isEnabled() const = 0;
        virtual bool isDisabled() const = 0;
        virtual void enable() = 0;
        virtual void disable() = 0;
        virtual void setLevel(bool level) = 0;
        virtual bool getLevel() const = 0;
        
        bool getLevelEnabled() const {
            return enableLevel_;
        };
        
        bool getLevelDisabled() const {
            return !enableLevel_;
        };
        
        bool toggle() {
            bool level = !getLevel();
            setLevel(level);
            return level;
        };
        
        uint8_t getPin() const {
            return gpio_;
        };

        IoMode getIoMode() const {
            return ioMode_;
        };

        PullMode getPullMode() const {
            return pullMode_;
        };
        
    protected:
        uint8_t gpio_ {0};
        bool enableLevel_ {1};
        IoMode ioMode_ {IoMode::Input_Output};
        PullMode pullMode_ {PullMode::Floating};
    };
} // namespace Generic
} // namespace Stepper


#endif // STEPPER_PIN_HPP