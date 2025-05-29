#ifndef STEPPER_EVENT_H
#define STEPPER_EVENT_H

#include <esp_event.h>
#include <esp_task.h>

namespace Stepper
{
  ESP_EVENT_DECLARE_BASE(TaskEventBaseId);
  
  enum class TaskEventId : int32_t
  {
    ANY_EVENT = -1,
    NEW_TASK_EVENT,
    TASK_STARTED_EVENT,
    TASK_FEEDBACK_EVENT,
    TASK_FINISHED_EVENT,
    TASK_ABORTED_EVENT
  };

  static esp_event_loop_handle_t taskEventLoopHandle;

  esp_err_t taskEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority, BaseType_t taskCoreId = 1, uint32_t taskStackSize = 3072);
  esp_err_t taskEventLoopDelete(void);
  esp_err_t taskEventLoopHandlerRegister(TaskEventId eventId, esp_event_handler_t eventHandler, void* eventHandlerArg = nullptr);
  esp_err_t taskEventLoopHandlerUnregister(TaskEventId eventId, esp_event_handler_t eventHandler);
  esp_err_t taskEventLoopPost(TaskEventId eventId, const void* eventData, size_t eventDataSize, uint32_t waitMs = 0);
  esp_err_t taskEventLoopPostISR(TaskEventId eventId, const void* eventData, size_t eventDataSize, BaseType_t* taskUnblocked);


  ESP_EVENT_DECLARE_BASE(StopEventBaseId);

  enum class StopEventId : int32_t
  {
    ANY_EVENT = -1,
    RESUME_EVENT,
    PAUSE_EVENT,
    HALT_EVENT,
    FAST_STOP_EVENT,
    EMERGENCY_STOP_EVENT
  };

  static esp_event_loop_handle_t stopEventLoopHandle;

  esp_err_t stopEventLoopCreate(int32_t queueSize, UBaseType_t taskPriority, BaseType_t taskCoreId = 1, uint32_t taskStackSize = 3072);
  esp_err_t stopEventLoopDelete(void);
  esp_err_t stopEventLoopHandlerRegister(StopEventId eventId, esp_event_handler_t eventHandler, void* eventHandlerArg = nullptr);
  esp_err_t stopEventLoopHandlerUnregister(StopEventId eventId, esp_event_handler_t eventHandler);
  esp_err_t stopEventLoopPost(StopEventId eventId, const void* eventData, size_t eventDataSize, uint32_t waitMs = 0);
  esp_err_t stopEventLoopPostISR(StopEventId eventId, const void* eventData, size_t eventDataSize, BaseType_t* taskUnblocked);

}

#endif // STEPPER_EVENT_H