#ifndef STEPPER_HOMING_ESP32_HPP
#define STEPPER_HOMING_ESP32_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <functional>

#include "../Generic/Homing.hpp"

namespace Stepper {
namespace ESP32 {

class Homing : public Generic::Homing {
public:
    using HomingCallback = std::function<void(bool success)>;

    Homing();
    explicit Homing(const Config& config);
    ~Homing() override;

    // Asynchronous non-blocking homing routines executed in a background FreeRTOS task
    bool homeAsync(Generic::Axis& axis, const Config& config, HomingCallback callback = nullptr);
    bool homeAsync(Generic::Axis& axis, Strategy strategy, HomingCallback callback = nullptr);
    bool homeAsync(Generic::Axis& axis, HomingCallback callback = nullptr);
    bool homeMinMaxAsync(Generic::Axis& axis, HomingCallback callback = nullptr);
    bool homeMinMaxAsync(Generic::Axis& axis, const Config& config, HomingCallback callback = nullptr);
    bool homeMaxMinAsync(Generic::Axis& axis, HomingCallback callback = nullptr);
    bool homeMaxMinAsync(Generic::Axis& axis, const Config& config, HomingCallback callback = nullptr);

    // Block calling FreeRTOS task until homing completes or timeout expires
    bool waitForHoming(uint32_t timeoutMs = UINT32_MAX);

    // Abort active homing sequence
    void abortHoming();

protected:
    static void task(void* args);

    Generic::Axis* activeAxis_ {nullptr};
    TaskHandle_t homingTaskHandle_ {nullptr};
    SemaphoreHandle_t homingSemaphoreHandle_ {nullptr};
    HomingCallback homingCallback_ {nullptr};
    Config pendingConfig_;
    bool lastResult_ {false};
};

} // namespace ESP32
} // namespace Stepper

#endif // STEPPER_HOMING_ESP32_HPP
