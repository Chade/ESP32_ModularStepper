#ifndef DRIVER_GPIO_H
#define DRIVER_GPIO_H

#include <cstdint>

typedef int gpio_num_t;
typedef int gpio_mode_t;
typedef int esp_err_t;

#ifndef ESP_OK
#define ESP_OK 0
#endif

#ifndef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) (void)(x)
#endif

#define GPIO_MODE_DISABLE 0
#define GPIO_MODE_INPUT 1
#define GPIO_MODE_OUTPUT 2
#define GPIO_MODE_INPUT_OUTPUT 3

inline esp_err_t gpio_set_direction(gpio_num_t, gpio_mode_t) { return ESP_OK; }
inline uint32_t gpio_get_level(gpio_num_t) { return 0; }
inline esp_err_t gpio_set_level(gpio_num_t, uint32_t) { return ESP_OK; }

#endif // DRIVER_GPIO_H
