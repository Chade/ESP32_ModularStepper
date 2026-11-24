#ifndef STEPPER_EVENT_HPP
#define STEPPER_EVENT_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include <esp_task.h>

#include "../Generic/StopController.hpp"

namespace Stepper {
namespace ESP32 {

ESP_EVENT_DECLARE_BASE(TaskEventBaseId);

enum class TaskEventId : int32_t {
    ANY_EVENT = -1,
    NEW_TASK_EVENT,
    TASK_STARTED_EVENT,
    TASK_FEEDBACK_EVENT,
    TASK_FINISHED_EVENT,
    TASK_ABORTED_EVENT
};

esp_err_t taskEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority, BaseType_t taskCoreId = 1, uint32_t taskStackSize = 3072);
esp_err_t taskEventLoopDelete(void);
esp_err_t taskEventLoopHandlerRegister(TaskEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg = nullptr);
esp_err_t taskEventLoopHandlerUnregister(TaskEventId eventId, esp_event_handler_t eventHandler);
esp_err_t taskEventLoopHandlerRegisterInstance(TaskEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg, esp_event_handler_instance_t *instance);
esp_err_t taskEventLoopHandlerUnregisterInstance(TaskEventId eventId, esp_event_handler_instance_t instance);
esp_err_t taskEventLoopPost(TaskEventId eventId, const void *eventData, size_t eventDataSize, uint32_t waitMs = 0);
esp_err_t taskEventLoopPostISR(TaskEventId eventId, const void *eventData, size_t eventDataSize, BaseType_t *taskUnblocked);

ESP_EVENT_DECLARE_BASE(StopEventBaseId);

enum class StopEventId : int32_t {
    ANY_EVENT = -1,
    RESUME_EVENT,
    PAUSE_EVENT,
    HALT_EVENT,
    FAST_STOP_EVENT,
    EMERGENCY_STOP_EVENT,
    CLEAR_EMERGENCY_STOP_EVENT
};

struct StopEventData {
    static constexpr int16_t ALL_TARGETS = Generic::StopController::ALL_TARGETS;

    int16_t targetId {ALL_TARGETS};
    bool disableDriver {false};
    bool enableDriver {false};
    float deceleration {0.0f};
};

esp_err_t stopEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority = configMAX_PRIORITIES - 1, BaseType_t taskCoreId = 1, uint32_t taskStackSize = 3072);
esp_err_t stopEventLoopDelete(void);
esp_err_t stopEventLoopHandlerRegister(StopEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg = nullptr);
esp_err_t stopEventLoopHandlerUnregister(StopEventId eventId, esp_event_handler_t eventHandler);
esp_err_t stopEventLoopHandlerRegisterInstance(StopEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg, esp_event_handler_instance_t *instance);
esp_err_t stopEventLoopHandlerUnregisterInstance(StopEventId eventId, esp_event_handler_instance_t instance);
esp_err_t stopEventLoopPost(StopEventId eventId, const void *eventData, size_t eventDataSize, uint32_t waitMs = 0);
esp_err_t stopEventLoopPostISR(StopEventId eventId, const void *eventData, size_t eventDataSize, BaseType_t *taskUnblocked);

// Bridge between ESP32 StopEventLoop and Generic::StopController
esp_err_t stopEventLoopAttachController(Generic::StopController& controller);
esp_err_t stopEventLoopDetachController();
esp_err_t stopEventPost(StopEventId eventId, const StopEventData& data = {}, uint32_t waitMs = 0);
esp_err_t stopEventPostISR(StopEventId eventId, const StopEventData& data, BaseType_t* taskUnblocked);

} // namespace ESP32
} // namespace Stepper

#endif // STEPPER_EVENT_HPP
