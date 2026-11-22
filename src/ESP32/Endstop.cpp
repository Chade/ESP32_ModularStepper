#include "Endstop.hpp"
#include <esp_timer.h>

namespace Stepper {
namespace ESP32 {

Endstop::Endstop(Generic::Pin& pin)
        : Generic::Endstop(pin) {
    gpio_num_t gpioNum = static_cast<gpio_num_t>(pin_.getPin());

    eventQueue_ = xQueueCreate(16, sizeof(Edge));

    xTaskCreatePinnedToCore(
        Endstop::task,
        "StepperEndstop",
        2048,
        this,
        configMAX_PRIORITIES - 2,
        &taskHandle_,
        1
    );

    // Ignore error if already installed
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gpio_set_intr_type(gpioNum, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpioNum, Endstop::isrHandler, this));
    // Default to disabled until enableInterrupt() is called
    ESP_ERROR_CHECK(gpio_intr_disable(gpioNum));
    enabled_ = false;
}

Endstop::~Endstop() {
    gpio_num_t gpioNum = static_cast<gpio_num_t>(pin_.getPin());
    gpio_intr_disable(gpioNum);
    gpio_isr_handler_remove(gpioNum);

    vTaskDelete(taskHandle_);
    vQueueDelete(eventQueue_);
}

void Endstop::enableInterrupt() {
    if (!enabled_) {
        ESP_ERROR_CHECK(gpio_intr_enable(static_cast<gpio_num_t>(pin_.getPin())));
        enabled_ = true;
    }
}

void Endstop::disableInterrupt() {
    if (enabled_) {
        ESP_ERROR_CHECK(gpio_intr_disable(static_cast<gpio_num_t>(pin_.getPin())));
        enabled_ = false;
    }
}

void IRAM_ATTR Endstop::isrHandler(void* args) {
    Endstop* self = static_cast<Endstop*>(args);
    if (!self->enabled_) {
        return;
    }

    uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    if (self->debounce_us_ > 0 && (now - self->lastInterruptTimeUs_) < self->debounce_us_) {
        return;
    }
    self->lastInterruptTimeUs_ = now;

    int level = gpio_get_level(static_cast<gpio_num_t>(self->pin_.getPin()));
    Edge edge = (level != 0) ? Edge::Rising : Edge::Falling;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(self->eventQueue_, &edge, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Endstop::task(void* args) {
    Endstop* self = static_cast<Endstop*>(args);
    Edge edge;
    for (;;) {
        if (xQueueReceive(self->eventQueue_, &edge, portMAX_DELAY) == pdTRUE) {
            self->triggerCallback(edge);
        }
    }
}

} // namespace ESP32
} // namespace Stepper
