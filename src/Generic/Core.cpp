#include "Core.hpp"
#include "Log.hpp"

namespace Stepper {

Core::Core() {
    LOG_INFO("Create task queue");
    m_taskQueueHandle = xQueueCreate(taskQueueSize, sizeof(struct Task));
    assert(m_taskQueueHandle != 0);

    LOG_INFO("Create task event loop");
    ESP_ERROR_CHECK(taskEventLoopCreate(taskEventLoopQueueSize, 2));

    LOG_INFO("Create stop event loop");
    ESP_ERROR_CHECK(stopEventLoopCreate(stopEventLoopQueueSize, 2));
}

Core::~Core() {
    stop();
    ESP_ERROR_CHECK(taskEventLoopDelete());
    ESP_ERROR_CHECK(stopEventLoopDelete());
}

void Core::start() {
    LOG_INFO("Register to task event");
    ESP_ERROR_CHECK(taskEventLoopHandlerRegister(TaskEventId::ANY_EVENT, taskEventCallback, this));

    LOG_INFO("Register to stop event");
    ESP_ERROR_CHECK(stopEventLoopHandlerRegister(StopEventId::ANY_EVENT, stopEventCallback, this));

    if (m_taskHandle == nullptr) {
        LOG_INFO("Create task runner");
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
    else {
        LOG_ERROR("Task runner already created!");
    }
}

void Core::stop() {
    LOG_INFO("Unregister from task event");
    ESP_ERROR_CHECK(taskEventLoopHandlerUnregister(TaskEventId::ANY_EVENT, taskEventCallback));

    LOG_INFO("Unregister from stop event");
    ESP_ERROR_CHECK(stopEventLoopHandlerUnregister(StopEventId::ANY_EVENT, stopEventCallback));

    // Signal task runnner to stop
    m_taskRunnerStop = true;

    // Wait for taskrunner to finish
    int64_t startTime = esp_timer_get_time();
    while (m_taskHandle != nullptr) {
        int64_t timeElapsed = esp_timer_get_time() - startTime;
        if (timeElapsed < taskRunnerStopTimeout_us) {
            LOG_INFO("Waiting for task runner to stop");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else {
            LOG_INFO("Force task runner to stop");
            vTaskDelete(m_taskHandle);
            m_taskHandle = nullptr;
        }
    }
}

bool Core::hasActiveTask() {
    return m_activeTask.getState() != Task::TaskState::UNDEFINED;
}

void Core::taskRunner(void *args) {
    Core *core = static_cast<Core *>(args);
    Task task;

    while (!core->m_taskRunnerStop) {

        if (core->hasActiveTask()) {
            //
        }
        else {
            if (xQueueReceive(core->m_taskQueueHandle, &task, pdMS_TO_TICKS(1000)) == pdTRUE) {
                core->m_activeTask = task;
                core->m_activeTask.setState(Task::TaskState::ACTIVE);
            }
        }
    }

    LOG_ERROR("Task runner aborted!");
    vTaskDelete(nullptr);
    core->m_taskHandle = nullptr;
}

void Core::taskEventCallback(void *eventArgs, esp_event_base_t eventBase, int32_t eventId, void *eventData) {
    Core *core = static_cast<Core *>(eventArgs);

    switch (static_cast<TaskEventId>(eventId)) {
        case TaskEventId::ANY_EVENT:
            LOG_INFO("Received new %s|ANY_EVENT", eventBase);
            break;
        case TaskEventId::NEW_TASK_EVENT:
            LOG_INFO("Received new %s|NEW_TASK_EVENT", eventBase);
            if (xQueueSendToBack(core->m_taskQueueHandle, eventData, 0) != pdPASS)
            {
                LOG_ERROR("Could not enqueue new task");
            }
            break;
        case TaskEventId::TASK_STARTED_EVENT:
            LOG_INFO("Received new %s|TASK_STARTED_EVENT", eventBase);
            break;
        case TaskEventId::TASK_FEEDBACK_EVENT:
            LOG_INFO("Received new %s|TASK_FEEDBACK_EVENT", eventBase);
            break;
        case TaskEventId::TASK_FINISHED_EVENT:
            LOG_INFO("Received new %s|TASK_FINISHED_EVENT", eventBase);
            break;
        case TaskEventId::TASK_ABORTED_EVENT:
            LOG_INFO("Received new %s|TASK_ABORTED_EVENT", eventBase);
            break;
        default:
            LOG_ERROR("Received unknown event %s|%i", eventBase, eventId);
    }
}

void Core::stopEventCallback(void *eventArgs, esp_event_base_t eventBase, int32_t eventId, void *eventData) {
    Core *core = static_cast<Core *>(eventArgs);

    switch (static_cast<StopEventId>(eventId)) {
        case StopEventId::ANY_EVENT:
            LOG_INFO("Received new %s|ANY_EVENT", eventBase);
            break;
        case StopEventId::RESUME_EVENT:
            LOG_INFO("Received new %s|RESUME_EVENT", eventBase);
            break;
        case StopEventId::PAUSE_EVENT:
            LOG_INFO("Received new %s|PAUSE_EVENT", eventBase);
            break;
        case StopEventId::HALT_EVENT:
            LOG_INFO("Received new %s|HALT_EVENT", eventBase);
            break;
        case StopEventId::FAST_STOP_EVENT:
            LOG_INFO("Received new %s|FAST_STOP_EVENT", eventBase);
            break;
        case StopEventId::EMERGENCY_STOP_EVENT:
            LOG_INFO("Received new %s|EMERGENCY_STOP_EVENT", eventBase);
            break;
        default:
            LOG_ERROR("Received unknown event %s|%i", eventBase, eventId);
    }
}

} // namespace Stepper