#ifndef STEPPER_CORE_H
#define STEPPER_CORE_H

#include "StepperEvent.h"
#include "StepperTask.h"
#include <deque>
#include <esp_log.h>
#include <esp_timer.h>

namespace Stepper
{
    class Core
    {
    public:
        Core();
        ~Core();

        void start();
        void stop();

        bool hasActiveTask();

    protected:
        const int32_t taskQueueSize = 10;
        const int32_t taskEventLoopQueueSize = 10;
        const int32_t stopEventLoopQueueSize = 10;
        const int32_t taskRunnerStopTimeout_us = 5000000;

        static void taskRunner(void *args);
        static void taskEventCallback(void *eventArgs, esp_event_base_t eventBase, int32_t eventId, void *eventData);
        static void stopEventCallback(void *eventArgs, esp_event_base_t eventBase, int32_t eventId, void *eventData);

        TaskHandle_t m_taskHandle{nullptr};
        QueueHandle_t m_taskQueueHandle{nullptr};

        CoreTask m_activeTask;
        bool m_taskRunnerStop{false};
    };
}

#endif // STEPPER_CORE_H