#ifndef STEPPER_GENERATOR_ESP32_HPP
#define STEPPER_GENERATOR_ESP32_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "../Generic/Generator.hpp"
#include "../Generic/Helper.hpp"

namespace Stepper {
namespace ESP32 {
    class Generator : public Generic::Generator {
    public:
        Generator() = delete;
        explicit Generator(Generic::Driver& driver);
        virtual ~Generator() override;

        State waitForState(State state, uint32_t timeout_ms = UINT32_MAX) const override;

    private:
        void updateStateEvent(State state) const override;
        void IRAM_ATTR updateStateEventFromISR(State state, BaseType_t* pxHigherPriorityTaskWoken = nullptr) const;

        EventGroupHandle_t eventGroupHandle_ {nullptr};
    };
} // namespace ESP32
} // namespace Stepper

#endif //STEPPER_GENERATOR_ESP32_HPP