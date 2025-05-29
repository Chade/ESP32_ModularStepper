#include "freertos/projdefs.h"
#include "StepperCore.h"

namespace Stepper
{
  Core::Core()
  {
    ESP_LOGI("Core", "Create task queue");
    m_taskQueueHandle = xQueueCreate(taskQueueSize, sizeof(struct Task));
    assert(m_taskQueueHandle != 0);

    ESP_LOGI("Core", "Create task event loop");
    ESP_ERROR_CHECK(taskEventLoopCreate(taskEventLoopQueueSize, uxTaskPriorityGet(NULL) + 1));

    ESP_LOGI("Core", "Create stop event loop");
    ESP_ERROR_CHECK(stopEventLoopCreate(stopEventLoopQueueSize, uxTaskPriorityGet(NULL) + 1));
  }

  Core::~Core()
  {
    stop();
    ESP_ERROR_CHECK(taskEventLoopDelete());
    ESP_ERROR_CHECK(stopEventLoopDelete());
  }

  

  void Core::start()
  {
    ESP_LOGI("Core", "Register to task event");
    ESP_ERROR_CHECK(taskEventLoopHandlerRegister(TaskEventId::ANY_EVENT, taskEventCallback, this));

    ESP_LOGI("Core", "Register to stop event");
    ESP_ERROR_CHECK(stopEventLoopHandlerRegister(StopEventId::ANY_EVENT, stopEventCallback, this));

    if (m_taskHandle == nullptr)
    {
      ESP_LOGI("Core", "Create task runner");
      //disableCore1WDT(); // Should not be necessary, as we should not add too much load to one core
      xTaskCreatePinnedToCore(
          Core::taskRunner,            /* Task function */
          "CoreTask",                  /* Task name (max. 16 characters by default) */
          3072,                        /* Stack size in bytes */
          this,                        /* Parameter passed as input of the task */
          uxTaskPriorityGet(NULL) + 2, /* Task priority */
          &m_taskHandle,               /* Task handle */
          1                            /* CPU core to use */
      );
    }
    else
    {
      ESP_LOGE("Core", "Task runner already created!");
    }
  }

  void Core::stop()
  {
    ESP_LOGI("Core", "Unregister from task event");
    ESP_ERROR_CHECK(taskEventLoopHandlerUnregister(TaskEventId::ANY_EVENT, taskEventCallback));

    ESP_LOGI("Core", "Unregister from stop event");
    ESP_ERROR_CHECK(stopEventLoopHandlerUnregister(StopEventId::ANY_EVENT, stopEventCallback));

    // Signal task runnner to stop
    m_taskRunnerStop = true;

    // Wait for taskrunner to finish
    int64_t startTime = esp_timer_get_time();
    while (m_taskHandle != nullptr)
    {
      int64_t timeElapsed = esp_timer_get_time() - startTime;
      if (timeElapsed < taskRunnerStopTimeout_us)
      {
        ESP_LOGI("Core", "Waiting for task runner to stop");
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      else {
        ESP_LOGI("Core", "Force task runner to stop");
        vTaskDelete(m_taskHandle);
        m_taskHandle = nullptr;
      }
    }
  }

  bool Core::hasActiveTask()
  {
    return m_activeTask.getState() != TaskState::UNDEFINED;
  }


  void Core::taskRunner(void* args)
  {
    Core* core = static_cast<Core*>(args);
    Task task;

    while (!core->m_taskRunnerStop)
    {

      if (core->hasActiveTask())
      {
        //

      }
      else
      {
        if (xQueueReceive(core->m_taskQueueHandle, &task, pdMS_TO_TICKS(1000) == pdTRUE))
        {
          core->m_activeTask = task;
          core->m_activeTask.setState(TaskState::ACTIVE);
        }
      }
    }

    ESP_LOGE("Core", "Task runner aborted!");
    vTaskDelete(nullptr);
    core->m_taskHandle = nullptr;
  }

  void Core::taskEventCallback(void* eventArgs, esp_event_base_t eventBase, int32_t eventId, void* eventData)
  {
    Core* core = static_cast<Core*>(eventArgs);

    switch (static_cast<TaskEventId>(eventId))
    {
      case TaskEventId::ANY_EVENT:
        ESP_LOGI("Core", "Received new %s|ANY_EVENT", eventBase);
        break;
      case TaskEventId::NEW_TASK_EVENT:
        ESP_LOGI("Core", "Received new %s|NEW_TASK_EVENT", eventBase);
        if (xQueueSendToBack(core->m_taskQueueHandle, eventData, 0) != pdPASS)
        {
          ESP_LOGE("Core", "Could not enqueue new task");
        }
        break;
      case TaskEventId::TASK_STARTED_EVENT:
        ESP_LOGI("Core", "Received new %s|TASK_STARTED_EVENT", eventBase);
        break;
      case TaskEventId::TASK_FEEDBACK_EVENT:
        ESP_LOGI("Core", "Received new %s|TASK_FEEDBACK_EVENT", eventBase);
        break;
      case TaskEventId::TASK_FINISHED_EVENT:
        ESP_LOGI("Core", "Received new %s|TASK_FINISHED_EVENT", eventBase);
        break;
      case TaskEventId::TASK_ABORTED_EVENT:
        ESP_LOGI("Core", "Received new %s|TASK_ABORTED_EVENT", eventBase);
        break;
      default:
        ESP_LOGE("Core", "Received unknown event %s|%i", eventBase, eventId);
    }
  }

  void Core::stopEventCallback(void* eventArgs, esp_event_base_t eventBase, int32_t eventId, void* eventData)
  {
    Core* core = static_cast<Core*>(eventArgs);

    switch (static_cast<StopEventId>(eventId))
    {
      case StopEventId::ANY_EVENT:
        ESP_LOGI("Core", "Received new %s|ANY_EVENT", eventBase);
        break;
      case StopEventId::RESUME_EVENT:
        ESP_LOGI("Core", "Received new %s|RESUME_EVENT", eventBase);
        break;
      case StopEventId::PAUSE_EVENT:
        ESP_LOGI("Core", "Received new %s|PAUSE_EVENT", eventBase);
        break;
      case StopEventId::HALT_EVENT:
        ESP_LOGI("Core", "Received new %s|HALT_EVENT", eventBase);
        break;
      case StopEventId::FAST_STOP_EVENT:
        ESP_LOGI("Core", "Received new %s|FAST_STOP_EVENT", eventBase);
        break;
      case StopEventId::EMERGENCY_STOP_EVENT:
        ESP_LOGI("Core", "Received new %s|EMERGENCY_STOP_EVENT", eventBase);
        break;
      default:
        ESP_LOGE("Core", "Received unknown event %s|%i", eventBase, eventId);
    }
  }


/*
Calculate the steps needed to reach the desired steps/s velocity:

steps_needed = (v1^2 - v0^2) / 2 * a

The time to finish the movement is calculated as:

t = (v1 - v0) / a

The time needs to be divided by the needed steps



We now know hom many steps we need to reach final velocity.
We now have to calculate the time between steps.
For a given constant speed v in steps/s the time between two steps is 1/v
For an accelerated movement, the time difference between steps needs to be adapted

v(t) = a*t
s(t) = v0*t + 0.5*a*t^2
x(t) = x0 + v0*t + 0.5*a*t^2

t1 = t0 * (1 - a * t0^2)
t1 = t0 - a * t0^3
*/


/*
void ESP_FlexyStepper::setSpeedInStepsPerSecond(float speedInStepsPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
  desiredPeriod_InUSPerStep = 1000000.0 / desiredSpeed_InStepsPerSecond;
}

void ESP_FlexyStepper::setAccelerationInStepsPerSecondPerSecond(
    float accelerationInStepsPerSecondPerSecond)
{
  acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
  acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;

  periodOfSlowestStep_InUS =
      1000000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);
  minimumPeriodForAStoppedMotion = periodOfSlowestStep_InUS / 2.8;
}

  //
  // determine the number of steps needed to go from the current speed down to a
  // velocity of 0, Steps = Velocity^2 / (2 * Deceleration)
  //
  currentStepPeriodSquared = currentStepPeriod_InUS * currentStepPeriod_InUS;
  decelerationDistance_InSteps = (long)round(
      5E11 / (deceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared));

// NextStepPeriod = CurrentStepPeriod(1 + acceleration * CurrentStepPeriod^2)
// NextStepPeriod = CurrentStepPeriod(1 - deceleration * CurrentStepPeriod^2)
*/


}