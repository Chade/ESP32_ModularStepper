#ifndef FREERTOS_H
#define FREERTOS_H

#include <cstdint>

typedef void* TaskHandle_t;
typedef int BaseType_t;
typedef int portMUX_TYPE;

#define portMUX_INITIALIZER_UNLOCKED 0
#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define pdFAIL 0
#define portMAX_DELAY 0xFFFFFFFF

inline void taskENTER_CRITICAL(portMUX_TYPE*) {}
inline void taskEXIT_CRITICAL(portMUX_TYPE*) {}
inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}

#endif // FREERTOS_H
