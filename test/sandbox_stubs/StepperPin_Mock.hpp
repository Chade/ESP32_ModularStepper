#ifndef STEPPER_PIN_MOCK_HPP
#define STEPPER_PIN_MOCK_HPP

#include "Generic/Pin.hpp"

namespace Stepper {
namespace Mock {
    class Pin : public Stepper::Generic::Pin {
    public:
        Pin()
                : Stepper::Generic::Pin(0) {
            level_ = !enableLevel_;
        };

        Pin(int8_t pin, IoMode mode = IoMode::Input_Output, PullMode pullMode = PullMode::Floating) 
                : Stepper::Generic::Pin(pin, mode, pullMode) {
            level_ = !enableLevel_;
        };

        Pin(uint8_t pin, bool inverse, IoMode mode = IoMode::Input_Output, PullMode pullMode = PullMode::Floating)
                : Stepper::Generic::Pin(pin, inverse, mode, pullMode) {
            level_ = !enableLevel_;
        };

        void setIoMode(IoMode mode) override {
            ioMode_ = mode;
        };

        void setPullMode(PullMode pullMode) override {
            pullMode_ = pullMode;
        };

        bool isEnabled() const override {
            return (level_ == enableLevel_);
        };

        bool isDisabled() const override {
            return (level_ != enableLevel_);
        };

        void enable() override {
            level_ = enableLevel_;
        };

        void disable() override {
            level_ = !enableLevel_;
        };

        void setLevel(bool level) override {
            level_ = level;
        };

        bool getLevel() const override {
            return level_;
        };

    private:
        bool level_ {false};
    };
} // namespace Mock
} // namespace Stepper

#endif // STEPPER_PIN_MOCK_HPP