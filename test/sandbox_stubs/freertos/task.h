#ifndef FREERTOS_TASK_H
#define FREERTOS_TASK_H

#include "FreeRTOS.h"

typedef enum {
    eNoAction = 0,
    eSetBits,
    eIncrement,
    eSetValueWithOverwrite,
    eSetValueWithoutOverwrite
} eNotifyAction;

inline void vTaskDelay(uint32_t) {}
inline void vTaskDelete(TaskHandle_t) {}
inline BaseType_t xTaskCreatePinnedToCore(void(*)(void*), const char*, uint32_t, void*, int, TaskHandle_t*, int) { return pdPASS; }
inline BaseType_t xTaskNotify(TaskHandle_t, uint32_t, eNotifyAction) { return pdPASS; }
inline BaseType_t xTaskNotifyFromISR(TaskHandle_t, uint32_t, eNotifyAction, BaseType_t*) { return pdPASS; }
inline BaseType_t xTaskNotifyGive(TaskHandle_t) { return pdPASS; }
inline BaseType_t xTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t*) { return pdPASS; }
inline uint32_t ulTaskNotifyTake(BaseType_t, uint32_t) { return 0; }
inline BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t*, uint32_t) { return pdFAIL; }

#endif // FREERTOS_TASK_H
