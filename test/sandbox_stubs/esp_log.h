#ifndef ESP_LOG_H
#define ESP_LOG_H

#include <cstdio>

typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE
} esp_log_level_t;

inline void esp_log_level_set(const char*, esp_log_level_t) {}
inline void esp_rom_delay_us(uint32_t us) { (void)us; }

#ifndef ESP_LOG_LEVEL_LOCAL
#define ESP_LOG_LEVEL_LOCAL(level, tag, fmt, ...) do { std::printf(fmt __VA_OPT__(,) __VA_ARGS__); std::printf("\n"); } while(0)
#endif

#ifndef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) std::printf("[INFO] [%s] " fmt "\n", tag __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) std::printf("[WARN] [%s] " fmt "\n", tag __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) std::printf("[ERROR] [%s] " fmt "\n", tag __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) std::printf("[DEBUG] [%s] " fmt "\n", tag __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) std::printf("[VERB] [%s] " fmt "\n", tag __VA_OPT__(,) __VA_ARGS__)
#endif

#ifndef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) (void)(x)
#endif

#endif // ESP_LOG_H
