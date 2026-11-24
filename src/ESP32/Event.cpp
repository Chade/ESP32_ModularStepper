#include "Event.hpp"
#include "../Generic/Helper.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {

namespace {

esp_event_loop_handle_t taskEventLoopHandle {nullptr};
esp_event_loop_handle_t stopEventLoopHandle {nullptr};
esp_event_handler_instance_t stopBridgeInstance {nullptr};

void stopBridgeHandler(void* eventArg, esp_event_base_t /*eventBase*/, int32_t eventId, void* eventData) {
    auto* controller = static_cast<Generic::StopController*>(eventArg);
    if (controller == nullptr) {
        return;
    }

    const StopEventData data = (eventData != nullptr)
                                   ? *static_cast<const StopEventData*>(eventData)
                                   : StopEventData{};

    const bool broadcast = (data.targetId == StopEventData::ALL_TARGETS);

    switch (static_cast<StopEventId>(eventId)) {
        case StopEventId::RESUME_EVENT:
            if (broadcast) {
                controller->resumeAll();
            } else {
                controller->resume(data.targetId);
            }
            break;

        case StopEventId::PAUSE_EVENT:
        case StopEventId::HALT_EVENT:
            if (broadcast) {
                controller->pauseAll();
            } else {
                controller->pause(data.targetId);
            }
            break;

        case StopEventId::FAST_STOP_EVENT:
            if (broadcast) {
                controller->fastStopAll(data.deceleration);
            } else {
                controller->fastStop(data.targetId, data.deceleration);
            }
            break;

        case StopEventId::EMERGENCY_STOP_EVENT:
            if (broadcast) {
                controller->emergencyStopAll(data.disableDriver);
            } else {
                controller->emergencyStop(data.targetId, data.disableDriver);
            }
            break;

        case StopEventId::CLEAR_EMERGENCY_STOP_EVENT:
            if (broadcast) {
                controller->clearEmergencyStopAll(data.enableDriver);
            } else {
                controller->clearEmergencyStop(data.targetId, data.enableDriver);
            }
            break;

        case StopEventId::ANY_EVENT:
            break;

        default:
            LOG_WARN("Unknown StopEventId: %ld", static_cast<long>(eventId));
            break;
    }
}

} // namespace

ESP_EVENT_DEFINE_BASE(TaskEventBaseId);

esp_err_t taskEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority, BaseType_t taskCoreId, uint32_t taskStackSize) {
    if (taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_event_loop_args_t taskEventLoopArgs = {
        .queue_size = queueSize,
        .task_name = "TaskEventLoop",
        .task_priority = taskPriority,
        .task_stack_size = taskStackSize,
        .task_core_id = taskCoreId};

    return esp_event_loop_create(&taskEventLoopArgs, &taskEventLoopHandle);
}

esp_err_t taskEventLoopDelete(void) {
    if (!taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_event_loop_delete(taskEventLoopHandle);
    if (err != ESP_OK) {
        return err;
    }

    taskEventLoopHandle = nullptr;
    return ESP_OK;
}

esp_err_t taskEventLoopHandlerRegister(TaskEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg) {
    if (!taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_register_with(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventHandler, eventHandlerArg);
}

esp_err_t taskEventLoopHandlerUnregister(TaskEventId eventId, esp_event_handler_t eventHandler) {
    if (!taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_unregister_with(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventHandler);
}

esp_err_t taskEventLoopHandlerRegisterInstance(TaskEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg, esp_event_handler_instance_t *instance) {
    if (!taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_instance_register_with(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventHandler, eventHandlerArg, instance);
}

esp_err_t taskEventLoopHandlerUnregisterInstance(TaskEventId eventId, esp_event_handler_instance_t instance) {
    if (!taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_instance_unregister_with(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), instance);
}

esp_err_t taskEventLoopPost(TaskEventId eventId, const void *eventData, size_t eventDataSize, uint32_t waitMs) {
    if (!taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_post_to(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, pdMS_TO_TICKS(waitMs));
}

esp_err_t taskEventLoopPostISR(TaskEventId eventId, const void *eventData, size_t eventDataSize, BaseType_t *taskUnblocked) {
    if (!taskEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_isr_post_to(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, taskUnblocked);
}

ESP_EVENT_DEFINE_BASE(StopEventBaseId);

esp_err_t stopEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority, BaseType_t taskCoreId, uint32_t taskStackSize) {
    if (stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_event_loop_args_t stopEventLoopArgs = {
        .queue_size = queueSize,
        .task_name = "StopEventLoop",
        .task_priority = taskPriority,
        .task_stack_size = taskStackSize,
        .task_core_id = taskCoreId};

    return esp_event_loop_create(&stopEventLoopArgs, &stopEventLoopHandle);
}

esp_err_t stopEventLoopDelete(void) {
    if (!stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    if (stopBridgeInstance != nullptr) {
        esp_err_t detachErr = stopEventLoopDetachController();
        if (detachErr != ESP_OK) {
            return detachErr;
        }
    }

    esp_err_t err = esp_event_loop_delete(stopEventLoopHandle);
    if (err != ESP_OK) {
        return err;
    }

    stopEventLoopHandle = nullptr;
    return ESP_OK;
}

esp_err_t stopEventLoopHandlerRegister(StopEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg) {
    if (!stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_register_with(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventHandler, eventHandlerArg);
}

esp_err_t stopEventLoopHandlerUnregister(StopEventId eventId, esp_event_handler_t eventHandler) {
    if (!stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_unregister_with(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventHandler);
}

esp_err_t stopEventLoopHandlerRegisterInstance(StopEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg, esp_event_handler_instance_t *instance) {
    if (!stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_instance_register_with(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventHandler, eventHandlerArg, instance);
}

esp_err_t stopEventLoopHandlerUnregisterInstance(StopEventId eventId, esp_event_handler_instance_t instance) {
    if (!stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_instance_unregister_with(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), instance);
}

esp_err_t stopEventLoopPost(StopEventId eventId, const void *eventData, size_t eventDataSize, uint32_t waitMs) {
    if (!stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_post_to(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, pdMS_TO_TICKS(waitMs));
}

esp_err_t stopEventLoopPostISR(StopEventId eventId, const void *eventData, size_t eventDataSize, BaseType_t *taskUnblocked) {
    if (!stopEventLoopHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_isr_post_to(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, taskUnblocked);
}

esp_err_t stopEventLoopAttachController(Generic::StopController& controller) {
    if (!stopEventLoopHandle || stopBridgeInstance != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_event_handler_instance_register_with(
        stopEventLoopHandle,
        StopEventBaseId,
        ESP_EVENT_ANY_ID,
        stopBridgeHandler,
        &controller,
        &stopBridgeInstance);
}

esp_err_t stopEventLoopDetachController() {
    if (!stopEventLoopHandle || stopBridgeInstance == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_event_handler_instance_unregister_with(
        stopEventLoopHandle,
        StopEventBaseId,
        ESP_EVENT_ANY_ID,
        stopBridgeInstance);
    if (err == ESP_OK) {
        stopBridgeInstance = nullptr;
    }
    return err;
}

esp_err_t stopEventPost(StopEventId eventId, const StopEventData& data, uint32_t waitMs) {
    return stopEventLoopPost(eventId, &data, sizeof(StopEventData), waitMs);
}

esp_err_t stopEventPostISR(StopEventId eventId, const StopEventData& data, BaseType_t* taskUnblocked) {
    return stopEventLoopPostISR(eventId, &data, sizeof(StopEventData), taskUnblocked);
}

} // namespace ESP32
} // namespace Stepper
