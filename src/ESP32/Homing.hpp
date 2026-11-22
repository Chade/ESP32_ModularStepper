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
    explicit Homing(const Config& defaultConfig);
    ~Homing() override;

    // Asynchronous non-blocking homing routines executed in a background FreeRTOS task
    bool homeAsync(Generic::Axis& axis, const Config& config, HomingCallback callback = nullptr);
    bool homeAsync(Generic::Axis& axis, Generic::Axis::EndstopPosition pos = Generic::Axis::EndstopPosition::Min, HomingCallback callback = nullptr);
    bool homeAsync(Generic::Axis& axis, HomingCallback callback);

    // Block calling FreeRTOS task until homing completes or timeout expires
    bool waitForHoming(uint32_t timeoutMs = portMAX_DELAY);

    // Abort active homing sequence
    void abortHoming();

protected:
    static void homingTask(void* args);

    Generic::Axis* activeAxis_ {nullptr};
    TaskHandle_t homingTaskHandle_ {nullptr};
    SemaphoreHandle_t homingDoneSem_ {nullptr};
    HomingCallback homingCallback_ {nullptr};
    Config pendingConfig_;
    bool lastResult_ {false};
};

} // namespace ESP32
} // namespace Stepper

#endif // STEPPER_HOMING_ESP32_HPP
