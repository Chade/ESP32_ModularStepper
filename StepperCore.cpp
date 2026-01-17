#include "StepperCore.h"
#include "StepperLog.h"

namespace Stepper
{
    Core::Core()
    {
        esp_log_level_set(log_tag, ESP_LOG_INFO);

        ESP_LOGI(log_tag, "Create task queue");
        m_taskQueueHandle = xQueueCreate(taskQueueSize, sizeof(struct Task));
        assert(m_taskQueueHandle != 0);

        ESP_LOGI(log_tag, "Create task event loop");
        ESP_ERROR_CHECK(taskEventLoopCreate(taskEventLoopQueueSize, 2));

        ESP_LOGI(log_tag, "Create stop event loop");
        ESP_ERROR_CHECK(stopEventLoopCreate(stopEventLoopQueueSize, 2));
    }

    Core::~Core()
    {
        stop();
        ESP_ERROR_CHECK(taskEventLoopDelete());
        ESP_ERROR_CHECK(stopEventLoopDelete());
    }

    void Core::start()
    {
        ESP_LOGI(log_tag, "Register to task event");
        ESP_ERROR_CHECK(taskEventLoopHandlerRegister(TaskEventId::ANY_EVENT, taskEventCallback, this));

        ESP_LOGI(log_tag, "Register to stop event");
        ESP_ERROR_CHECK(stopEventLoopHandlerRegister(StopEventId::ANY_EVENT, stopEventCallback, this));

        if (m_taskHandle == nullptr)
        {
            ESP_LOGI(log_tag, "Create task runner");
            // disableCore1WDT(); // Should not be necessary, as we should not add too much load to one core
            xTaskCreatePinnedToCore(
                Core::taskRunner,            /* Task function */
                "CoreTask",                  /* Task name (max. 16 characters by default) */
                3072,                        /* Stack size in bytes */
                this,                        /* Parameter passed as input of the task */
                1,                           /* Task priority */
                &m_taskHandle,               /* Task handle */
                1                            /* CPU core to use */
            );
        }
        else
        {
            ESP_LOGE(log_tag, "Task runner already created!");
        }
    }

    void Core::stop()
    {
        ESP_LOGI(log_tag, "Unregister from task event");
        ESP_ERROR_CHECK(taskEventLoopHandlerUnregister(TaskEventId::ANY_EVENT, taskEventCallback));

        ESP_LOGI(log_tag, "Unregister from stop event");
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
                ESP_LOGI(log_tag, "Waiting for task runner to stop");
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            else
            {
                ESP_LOGI(log_tag, "Force task runner to stop");
                vTaskDelete(m_taskHandle);
                m_taskHandle = nullptr;
            }
        }
    }

    bool Core::hasActiveTask()
    {
        return m_activeTask.getState() != Task::TaskState::UNDEFINED;
    }

    void Core::taskRunner(void *args)
    {
        Core *core = static_cast<Core *>(args);
        Task task;

        while (!core->m_taskRunnerStop)
        {

            if (core->hasActiveTask())
            {
                //
            }
            else
            {
                if (xQueueReceive(core->m_taskQueueHandle, &task, pdMS_TO_TICKS(1000)) == pdTRUE)
                {
                    core->m_activeTask = task;
                    core->m_activeTask.setState(Task::TaskState::ACTIVE);
                }
            }
        }

        ESP_LOGE(log_tag, "Task runner aborted!");
        vTaskDelete(nullptr);
        core->m_taskHandle = nullptr;
    }

    void Core::taskEventCallback(void *eventArgs, esp_event_base_t eventBase, int32_t eventId, void *eventData)
    {
        Core *core = static_cast<Core *>(eventArgs);

        switch (static_cast<TaskEventId>(eventId))
        {
        case TaskEventId::ANY_EVENT:
            ESP_LOGI(log_tag, "Received new %s|ANY_EVENT", eventBase);
            break;
        case TaskEventId::NEW_TASK_EVENT:
            ESP_LOGI(log_tag, "Received new %s|NEW_TASK_EVENT", eventBase);
            if (xQueueSendToBack(core->m_taskQueueHandle, eventData, 0) != pdPASS)
            {
                ESP_LOGE(log_tag, "Could not enqueue new task");
            }
            break;
        case TaskEventId::TASK_STARTED_EVENT:
            ESP_LOGI(log_tag, "Received new %s|TASK_STARTED_EVENT", eventBase);
            break;
        case TaskEventId::TASK_FEEDBACK_EVENT:
            ESP_LOGI(log_tag, "Received new %s|TASK_FEEDBACK_EVENT", eventBase);
            break;
        case TaskEventId::TASK_FINISHED_EVENT:
            ESP_LOGI(log_tag, "Received new %s|TASK_FINISHED_EVENT", eventBase);
            break;
        case TaskEventId::TASK_ABORTED_EVENT:
            ESP_LOGI(log_tag, "Received new %s|TASK_ABORTED_EVENT", eventBase);
            break;
        default:
            ESP_LOGE(log_tag, "Received unknown event %s|%i", eventBase, eventId);
        }
    }

    void Core::stopEventCallback(void *eventArgs, esp_event_base_t eventBase, int32_t eventId, void *eventData)
    {
        Core *core = static_cast<Core *>(eventArgs);

        switch (static_cast<StopEventId>(eventId))
        {
        case StopEventId::ANY_EVENT:
            ESP_LOGI(log_tag, "Received new %s|ANY_EVENT", eventBase);
            break;
        case StopEventId::RESUME_EVENT:
            ESP_LOGI(log_tag, "Received new %s|RESUME_EVENT", eventBase);
            break;
        case StopEventId::PAUSE_EVENT:
            ESP_LOGI(log_tag, "Received new %s|PAUSE_EVENT", eventBase);
            break;
        case StopEventId::HALT_EVENT:
            ESP_LOGI(log_tag, "Received new %s|HALT_EVENT", eventBase);
            break;
        case StopEventId::FAST_STOP_EVENT:
            ESP_LOGI(log_tag, "Received new %s|FAST_STOP_EVENT", eventBase);
            break;
        case StopEventId::EMERGENCY_STOP_EVENT:
            ESP_LOGI(log_tag, "Received new %s|EMERGENCY_STOP_EVENT", eventBase);
            break;
        default:
            ESP_LOGE(log_tag, "Received unknown event %s|%i", eventBase, eventId);
        }
    }

}