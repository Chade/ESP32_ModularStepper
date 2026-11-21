#ifndef STEPPER_ENDSTOP_ESP32_HPP
#define STEPPER_ENDSTOP_ESP32_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>

#include "StepperEndstop.hpp"
#include "StepperPin_ESP32.hpp"

namespace Stepper {
namespace ESP32 {

class Endstop : public Generic::Endstop {
public:
    using ISRCallback = void (*)(Edge edge, void* user_ctx);

    Endstop() = delete;
    Endstop(Generic::Pin& pin);
    ~Endstop() override;

    void enableInterrupt() override;
    void disableInterrupt() override;

protected:
    static void IRAM_ATTR isrHandler(void* args);
    static void task(void* args);

    static inline uint32_t isrServiceRefCount_ {0};

    bool enabled_ {false};

    QueueHandle_t eventQueue_ {nullptr};
    TaskHandle_t taskHandle_ {nullptr};

    volatile uint32_t lastInterruptTimeUs_ {0};
};

} // namespace ESP32
} // namespace Stepper

#endif // STEPPER_ENDSTOP_ESP32_HPP
