#include "Homing.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {

Homing::Homing()
    : Generic::Homing() {
    homingDoneSem_ = xSemaphoreCreateBinary();
    if (homingDoneSem_ != nullptr) {
        xSemaphoreGive(homingDoneSem_);
    }
}

Homing::Homing(const Config& defaultConfig)
    : Generic::Homing(defaultConfig) {
    homingDoneSem_ = xSemaphoreCreateBinary();
    if (homingDoneSem_ != nullptr) {
        xSemaphoreGive(homingDoneSem_);
    }
}

Homing::~Homing() {
    abortHoming();
    if (homingDoneSem_ != nullptr) {
        vSemaphoreDelete(homingDoneSem_);
        homingDoneSem_ = nullptr;
    }
}

bool Homing::homeAsync(Generic::Axis& axis, const Config& config, HomingCallback callback) {
    if (isHoming() || homingTaskHandle_ != nullptr) {
        LOG_WARN("Homing is already in progress");
        return false;
    }

    if (!axis.hasEndstop(config.endstop)) {
        LOG_WARN("Cannot start async homing: Target endstop not attached");
        return false;
    }

    activeAxis_ = &axis;
    homingCallback_ = std::move(callback);
    pendingConfig_ = config;
    lastResult_ = false;

    // Reset completion semaphore before starting
    if (homingDoneSem_ != nullptr) {
        xSemaphoreTake(homingDoneSem_, 0);
    }

    BaseType_t res = xTaskCreatePinnedToCore(
        Homing::homingTask,
        "AxisHoming",
        4096,
        this,
        configMAX_PRIORITIES - 3,
        &homingTaskHandle_,
        1 // Run on core 1
    );

    if (res != pdPASS) {
        LOG_ERROR("Failed to create FreeRTOS homing task");
        activeAxis_ = nullptr;
        if (homingDoneSem_ != nullptr) {
            xSemaphoreGive(homingDoneSem_);
        }
        return false;
    }

    return true;
}

bool Homing::homeAsync(Generic::Axis& axis, Generic::Axis::EndstopPosition pos, HomingCallback callback) {
    Config cfg = config_;
    cfg.endstop = pos;
    return homeAsync(axis, cfg, std::move(callback));
}

bool Homing::homeAsync(Generic::Axis& axis, HomingCallback callback) {
    return homeAsync(axis, config_.endstop, std::move(callback));
}

bool Homing::waitForHoming(uint32_t timeoutMs) {
    if (!isHoming() && homingTaskHandle_ == nullptr) {
        return lastResult_;
    }

    if (homingDoneSem_ == nullptr) {
        return false;
    }

    TickType_t ticks = (timeoutMs == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    if (xSemaphoreTake(homingDoneSem_, ticks) == pdTRUE) {
        xSemaphoreGive(homingDoneSem_);
        return lastResult_;
    }

    LOG_WARN("waitForHoming timed out");
    return false;
}

void Homing::abortHoming() {
    if (isHoming() || homingTaskHandle_ != nullptr) {
        LOG_INFO("Aborting active homing sequence");

        if (homingTaskHandle_ != nullptr) {
            vTaskDelete(homingTaskHandle_);
            homingTaskHandle_ = nullptr;
        }

        if (activeAxis_ != nullptr) {
            activeAxis_->stop();
            Generic::Endstop* targetEndstop = activeAxis_->getEndstop(pendingConfig_.endstop);
            if (targetEndstop != nullptr) {
                targetEndstop->unregisterCallback();
            }
            restoreLimitProtection(*activeAxis_, pendingConfig_.endstop);
            activeAxis_->setHoming(false);
            activeAxis_ = nullptr;
        }

        state_ = State::Failed;
        lastResult_ = false;

        if (homingCallback_) {
            auto cb = std::move(homingCallback_);
            cb(false);
        }

        if (homingDoneSem_ != nullptr) {
            xSemaphoreGive(homingDoneSem_);
        }
    }
}

void Homing::homingTask(void* args) {
    Homing* self = static_cast<Homing*>(args);

    LOG_INFO("FreeRTOS homing task started");
    bool success = false;
    if (self->activeAxis_ != nullptr) {
        success = self->home(*self->activeAxis_, self->pendingConfig_);
    }

    self->lastResult_ = success;
    self->activeAxis_ = nullptr;

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
