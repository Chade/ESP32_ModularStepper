#ifndef LOGGER_HPP
#define LOGGER_HPP


#if defined(ESP_PLATFORM) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

// =============================================================================
// ESP32 Build Target: Use ESP-IDF Logging Subsystem
// =============================================================================

// Map Arduino's CORE_DEBUG_LEVEL to ESP-IDF's LOG_LOCAL_LEVEL if not already set.
// This prevents ESP-IDF from stripping logs below ERROR at compile time.
// #ifndef LOG_LOCAL_LEVEL
//   #ifdef CORE_DEBUG_LEVEL
//     #define LOG_LOCAL_LEVEL CORE_DEBUG_LEVEL
//   #else
//     #define LOG_LOCAL_LEVEL 3 // ESP_LOG_INFO
//   #endif
// #endif

#include <Arduino.h>

#define LOG_LEVEL_NONE    ESP_LOG_NONE
#define LOG_LEVEL_ERROR   ESP_LOG_ERROR
#define LOG_LEVEL_WARN    ESP_LOG_WARN
#define LOG_LEVEL_INFO    ESP_LOG_INFO
#define LOG_LEVEL_DEBUG   ESP_LOG_DEBUG
#define LOG_LEVEL_VERBOSE ESP_LOG_VERBOSE

#define LOG_ERROR( format, ... )   ESP_LOGE("ERROR  ", format, ##__VA_ARGS__)
#define LOG_WARN( format, ... )    ESP_LOGW("WARN   ", format, ##__VA_ARGS__)
#define LOG_INFO( format, ... )    ESP_LOGI("INFO   ", format, ##__VA_ARGS__)
#define LOG_DEBUG( format, ... )   ESP_LOGD("DEBUG  ", format, ##__VA_ARGS__)
#define LOG_VERBOSE( format, ... ) ESP_LOGV("VERBOSE", format, ##__VA_ARGS__)

#undef ARDUHAL_LOG_FORMAT
#define ARDUHAL_LOG_FORMAT(letter, format)                                                                                                              \
  ARDUHAL_LOG_COLOR_##letter format ARDUHAL_LOG_RESET_COLOR "\n"

// Initialize runtime log level (needed because Arduino's initArduino sets level to ERROR by default)
#define LOG_LEVEL_INIT()          esp_log_level_set("*", (esp_log_level_t)LOG_LOCAL_LEVEL)
#define LOG_LEVEL_SET(tag, level) esp_log_level_set(tag, (esp_log_level_t)(level))


#else

// =============================================================================
// x86 / Host Build Target: Console Output (printf)
// =============================================================================

#include <cstdio>

#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"  //ERROR
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_YELLOW  "33"  //WARNING
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_MAGENTA "35"
#define LOG_COLOR_CYAN    "36"  //DEBUG
#define LOG_COLOR_GRAY    "37"  //VERBOSE
#define LOG_COLOR_WHITE   "38"  //INFO

#define LOG_COLOR(color)      "\033[0;" color "m"
#define LOG_COLOR_BOLD(color) "\033[1;" color "m"
#define LOG_COLOR_RESET       "\033[0m"

#define LOG_COLOR_ERROR    LOG_COLOR(LOG_COLOR_RED)
#define LOG_COLOR_WARN     LOG_COLOR(LOG_COLOR_YELLOW)
#define LOG_COLOR_INFO     LOG_COLOR(LOG_COLOR_WHITE)
#define LOG_COLOR_DEBUG    LOG_COLOR(LOG_COLOR_GREEN)
#define LOG_FORMAT(level, format)	LOG_COLOR_##level "[" #level "] " format LOG_COLOR_RESET "\n"

#define LOG_LEVEL_NONE    0
#define LOG_LEVEL_ERROR   1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_INFO    3
#define LOG_LEVEL_DEBUG   4
#define LOG_LEVEL_VERBOSE 5

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_INFO
#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR( format, ... )   std::printf(LOG_FORMAT(ERROR,   format), ##__VA_ARGS__)
#else
#define LOG_ERROR( format, ... ) \
    do {                     \
    } while (0)
#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN( format, ... )    std::printf(LOG_FORMAT(WARN,    format), ##__VA_ARGS__)
#else
#define LOG_WARN( format, ... ) \
    do {                     \
    } while (0)
#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO( format, ... )    std::printf(LOG_FORMAT(INFO,    format), ##__VA_ARGS__)
#else
#define LOG_INFO( format, ... ) \
    do {                     \
    } while (0)
#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_DEBUG
#define LOG_DEBUG( format, ... )   std::printf(LOG_FORMAT(DEBUG,   format), ##__VA_ARGS__)
#else
#define LOG_DEBUG( format, ... ) \
    do {                     \
    } while (0)
#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_VERBOSE
#define LOG_VERBOSE( format, ... ) std::printf(LOG_FORMAT(VERBOSE, format), ##__VA_ARGS__)
#else
#define LOG_VERBOSE( format, ... ) \
    do {                     \
    } while (0)
#endif

// No-ops for host builds
#define LOG_LEVEL_INIT()          ((void)0)
#define LOG_LEVEL_SET(tag, level) ((void)0)


#endif // Target selection

#endif // LOGGER_HPP
