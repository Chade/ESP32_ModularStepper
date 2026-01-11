#include "StepperEvent.h"
#include "StepperHelper.h"

namespace Stepper
{
    ESP_EVENT_DEFINE_BASE(TaskEventBaseId);

    esp_err_t taskEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority, BaseType_t taskCoreId, uint32_t taskStackSize)
    {
        if (taskEventLoopHandle)
        {
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

    esp_err_t taskEventLoopDelete(void)
    {
        if (!taskEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        esp_err_t err = esp_event_loop_delete(taskEventLoopHandle);

        if (err != ESP_OK)
        {
            return err;
        }

        taskEventLoopHandle = nullptr;

        return ESP_OK;
    }

    esp_err_t taskEventLoopHandlerRegister(TaskEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg)
    {
        if (!taskEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_handler_register_with(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventHandler, eventHandlerArg);
    }

    esp_err_t taskEventLoopHandlerUnregister(TaskEventId eventId, esp_event_handler_t eventHandler)
    {
        if (!taskEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_handler_unregister_with(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventHandler);
    }

    esp_err_t taskEventLoopPost(TaskEventId eventId, const void *eventData, size_t eventDataSize, uint32_t waitMs)
    {
        if (!taskEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_post_to(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, pdMS_TO_TICKS(waitMs));
    }

    esp_err_t taskEventLoopPostISR(TaskEventId eventId, const void *eventData, size_t eventDataSize, BaseType_t *taskUnblocked)
    {
        if (!taskEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_isr_post_to(taskEventLoopHandle, TaskEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, taskUnblocked);
    }

    ESP_EVENT_DEFINE_BASE(StopEventBaseId);

    esp_err_t stopEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority, BaseType_t taskCoreId, uint32_t taskStackSize)
    {
        if (stopEventLoopHandle)
        {
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

    esp_err_t stopEventLoopDelete(void)
    {
        if (!stopEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        esp_err_t err = esp_event_loop_delete(stopEventLoopHandle);

        if (err != ESP_OK)
        {
            return err;
        }

        stopEventLoopHandle = nullptr;

        return ESP_OK;
    }

    esp_err_t stopEventLoopHandlerRegister(StopEventId eventId, esp_event_handler_t eventHandler, void *eventHandlerArg)
    {
        if (!stopEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_handler_register_with(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventHandler, eventHandlerArg);
    }

    esp_err_t stopEventLoopHandlerUnregister(StopEventId eventId, esp_event_handler_t eventHandler)
    {
        if (!stopEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_handler_unregister_with(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventHandler);
    }

    esp_err_t stopEventLoopPost(StopEventId eventId, const void *eventData, size_t eventDataSize, uint32_t waitMs)
    {
        if (!stopEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_post_to(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, pdMS_TO_TICKS(waitMs));
    }

    esp_err_t stopEventLoopPostISR(StopEventId eventId, const void *eventData, size_t eventDataSize, BaseType_t *taskUnblocked)
    {
        if (!stopEventLoopHandle)
        {
            return ESP_ERR_INVALID_STATE;
        }

        return esp_event_isr_post_to(stopEventLoopHandle, StopEventBaseId, cast_enum_to_base(eventId), eventData, eventDataSize, taskUnblocked);
    }
}