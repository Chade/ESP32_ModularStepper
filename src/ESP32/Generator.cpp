#include "Generator.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {

    Generator::Generator(Generic::Driver& driver)
        : Generic::Generator(driver) {
        // Create event group
        eventGroupHandle_ = xEventGroupCreate();
    }

    Generator::~Generator() {
        // Delete event group
        if (eventGroupHandle_ != nullptr)
        {
            vEventGroupDelete(eventGroupHandle_);
        }
    }

    void Generator::updateStateEvent(State state) const {
        EventBits_t eventBits = static_cast<EventBits_t>(state);
        xEventGroupClearBits(eventGroupHandle_, 255);
        xEventGroupSetBits(eventGroupHandle_, eventBits);
    }

    void IRAM_ATTR Generator::updateStateEventFromISR(State state, BaseType_t* pxHigherPriorityTaskWoken) const {
        EventBits_t eventBits = static_cast<EventBits_t>(state);
        xEventGroupClearBitsFromISR(eventGroupHandle_, 255);
        xEventGroupSetBitsFromISR(eventGroupHandle_, eventBits, pxHigherPriorityTaskWoken);
    }

    State Generator::waitForState(State state, uint32_t timeout_ms) const {
        EventBits_t eventBits = static_cast<EventBits_t>(state);
        TickType_t ticksToWait = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

        return static_cast<State>(xEventGroupWaitBits(eventGroupHandle_, eventBits, pdFALSE, pdTRUE, ticksToWait));
    }
} // namespace Generic
} // namespace Stepper