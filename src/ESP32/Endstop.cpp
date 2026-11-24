#include <driver/gpio.h>
#include <esp_timer.h>

#include "Endstop.hpp"

namespace Stepper {
namespace ESP32 {

Endstop::Endstop(Generic::Pin& pin)
        : Generic::Endstop(pin) {

}

Endstop::~Endstop() {
    if (initialized_) {
        gpio_num_t gpioNum = static_cast<gpio_num_t>(pin_.getPin());
        gpio_intr_disable(gpioNum);
        gpio_isr_handler_remove(gpioNum);

        if (taskHandle_ != nullptr) {
            vTaskDelete(taskHandle_);
            taskHandle_ = nullptr;
        }
        if (eventQueue_ != nullptr) {
            vQueueDelete(eventQueue_);
            eventQueue_ = nullptr;
        }
        initialized_ = false;
        enabled_ = false;
    }
}

void Endstop::init() {
    if (initialized_) {
        return;
    }

    eventQueue_ = xQueueCreate(16, sizeof(Edge));
    
    xTaskCreatePinnedToCore(
        Endstop::task,
        "EndstopTask",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &taskHandle_,
        1
    );
    
    // Install GPIO ISR service once (ESP-IDF logs an error if called again)
    static bool isrServiceInstalled = false;
    if (!isrServiceInstalled) {
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(err);
        }
        isrServiceInstalled = true;
    }
    
    gpio_num_t gpioNum = static_cast<gpio_num_t>(pin_.getPin());
    ESP_ERROR_CHECK(gpio_set_intr_type(gpioNum, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpioNum, Endstop::isrHandler, this));
    // Default to disabled until enableInterrupt() is called
    ESP_ERROR_CHECK(gpio_intr_disable(gpioNum));

    enabled_ = false;
    initialized_ = true;
}

void Endstop::enableInterrupt() {
    if (!initialized_) {
        init();
    }
    if (!enabled_) {
        ESP_ERROR_CHECK(gpio_intr_enable(static_cast<gpio_num_t>(pin_.getPin())));
        enabled_ = true;
    }
}

void Endstop::disableInterrupt() {
    if (initialized_ && enabled_) {
        ESP_ERROR_CHECK(gpio_intr_disable(static_cast<gpio_num_t>(pin_.getPin())));
        enabled_ = false;
    }
}

void IRAM_ATTR Endstop::isrHandler(void* args) {
    Endstop* self = static_cast<Endstop*>(args);
    if (!self->enabled_) {
        return;
    }

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
            if (self->debounce_us_ > 0) {
                // Settling-based debounce: wait until no new edges arrive for debounce_us_,
                // then evaluate the final settled pin level so bounce edges are never lost.
                TickType_t debounceTicks = pdMS_TO_TICKS((self->debounce_us_ + 999) / 1000);
                if (debounceTicks == 0) {
                    debounceTicks = 1;
                }
                while (xQueueReceive(self->eventQueue_, &edge, debounceTicks) == pdTRUE) {
                    // Drain bouncing edges until the signal settles
                }
                int settledLevel = gpio_get_level(static_cast<gpio_num_t>(self->pin_.getPin()));
                edge = (settledLevel != 0) ? Edge::Rising : Edge::Falling;
            }
            self->triggerCallback(edge);
        }
    }
}

} // namespace ESP32
} // namespace Stepper
