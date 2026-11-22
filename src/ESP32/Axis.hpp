#ifndef STEPPER_AXIS_ESP32_HPP
#define STEPPER_AXIS_ESP32_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <functional>

#include "../Generic/Axis.hpp"
#include "Endstop.hpp"

namespace Stepper {
namespace ESP32 {

class Axis : public virtual Generic::Axis {
public:
    using HomingCallback = std::function<void(bool success)>;

    Axis() = delete;
    explicit Axis(Generic::Motor& motor);
    ~Axis() override;

    // Asynchronous non-blocking homing routines executed in a background FreeRTOS task
    bool homeAsync(const HomingConfig& config, HomingCallback callback = nullptr);
    bool homeAsync(EndstopPosition pos = EndstopPosition::Min, HomingCallback callback = nullptr);
    bool homeAsync(HomingCallback callback);

    // Block calling FreeRTOS task until homing completes or timeout expires
    bool waitForHoming(uint32_t timeoutMs = portMAX_DELAY);

    // Abort active homing sequence
    void abortHoming();

protected:
    static void homingTask(void* args);

    TaskHandle_t homingTaskHandle_ {nullptr};
    SemaphoreHandle_t homingDoneSem_ {nullptr};
    HomingCallback homingCallback_ {nullptr};
    HomingConfig pendingConfig_;
};

} // namespace ESP32
} // namespace Stepper

#endif // STEPPER_AXIS_ESP32_HPP
