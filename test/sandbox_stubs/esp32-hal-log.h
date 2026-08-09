#ifndef ESP32_HAL_LOG_H
#define ESP32_HAL_LOG_H

#define CORE_DEBUG_LEVEL 3
#define ARDUHAL_LOG_COLOR_E "[ERROR] "
#define ARDUHAL_LOG_COLOR_W "[WARN]  "
#define ARDUHAL_LOG_COLOR_I "[INFO]  "
#define ARDUHAL_LOG_COLOR_D "[DEBUG] "
#define ARDUHAL_LOG_COLOR_V "[VERBOSE] "
#define ARDUHAL_LOG_RESET_COLOR ""

inline const char* pathToFileName(const char* path) { return path; }

#endif // ESP32_HAL_LOG_H
