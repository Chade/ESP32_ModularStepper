#include "Axis.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {

Axis::Axis(Generic::Motor& motor)
    : Generic::Axis(motor) {
    homingDoneSem_ = xSemaphoreCreateBinary();
    if (homingDoneSem_ != nullptr) {
        xSemaphoreGive(homingDoneSem_);
    }
}

Axis::~Axis() {
    abortHoming();
    if (homingDoneSem_ != nullptr) {
        vSemaphoreDelete(homingDoneSem_);
        homingDoneSem_ = nullptr;
    }
}

bool Axis::homeAsync(const HomingConfig& config, HomingCallback callback) {
    if (isHoming_) {
        LOG_WARN("Homing is already in progress");
        return false;
    }

    if (!hasEndstop(config.endstop)) {
        LOG_WARN("Cannot start async homing: Target endstop not attached");
        return false;
    }

    homingCallback_ = std::move(callback);
    pendingConfig_ = config;

    // Reset completion semaphore before starting
    if (homingDoneSem_ != nullptr) {
        xSemaphoreTake(homingDoneSem_, 0);
    }

    BaseType_t res = xTaskCreatePinnedToCore(
        Axis::homingTask,
        "AxisHoming",
        4096,
        this,
        configMAX_PRIORITIES - 3,
        &homingTaskHandle_,
        1 // Run on core 1
    );

    if (res != pdPASS) {
        LOG_ERROR("Failed to create FreeRTOS homing task");
        if (homingDoneSem_ != nullptr) {
            xSemaphoreGive(homingDoneSem_);
        }
        return false;
    }

    return true;
}

bool Axis::homeAsync(EndstopPosition pos, HomingCallback callback) {
    HomingConfig cfg = homingConfig_;
    cfg.endstop = pos;
    return homeAsync(cfg, std::move(callback));
}

bool Axis::homeAsync(HomingCallback callback) {
    return homeAsync(homingConfig_.endstop, std::move(callback));
}

bool Axis::waitForHoming(uint32_t timeoutMs) {
    if (!isHoming_ && homingTaskHandle_ == nullptr) {
        return isHomed_;
    }

    if (homingDoneSem_ == nullptr) {
        return false;
    }

    TickType_t ticks = (timeoutMs == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    if (xSemaphoreTake(homingDoneSem_, ticks) == pdTRUE) {
        xSemaphoreGive(homingDoneSem_);
        return isHomed_;
    }

    LOG_WARN("waitForHoming timed out");
    return false;
}

void Axis::abortHoming() {
    if (isHoming_ || homingTaskHandle_ != nullptr) {
        LOG_INFO("Aborting active homing sequence");
        stop();

        if (homingTaskHandle_ != nullptr) {
            vTaskDelete(homingTaskHandle_);
            homingTaskHandle_ = nullptr;
        }

        isHoming_ = false;
        homingState_ = HomingState::Failed;

        if (homingCallback_) {
            auto cb = std::move(homingCallback_);
            cb(false);
        }

        if (homingDoneSem_ != nullptr) {
            xSemaphoreGive(homingDoneSem_);
        }
    }
}

void Axis::homingTask(void* args) {
    Axis* self = static_cast<Axis*>(args);

    LOG_INFO("FreeRTOS homing task started");
    bool success = self->home(self->pendingConfig_);

    if (self->homingCallback_) {
        auto cb = std::move(self->homingCallback_);
        cb(success);
    }

    if (self->homingDoneSem_ != nullptr) {
        xSemaphoreGive(self->homingDoneSem_);
    }

    self->homingTaskHandle_ = nullptr;
    LOG_INFO("FreeRTOS homing task completed (%s)", success ? "SUCCESS" : "FAILED");
    vTaskDelete(NULL);
}

} // namespace ESP32
} // namespace Stepper
