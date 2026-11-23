#include "Homing.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {

Homing::Homing() {
    homingSemaphoreHandle_ = xSemaphoreCreateBinary();
    if (homingSemaphoreHandle_ != nullptr) {
        xSemaphoreGive(homingSemaphoreHandle_);
    }
}


Homing::Homing(const Config& config)
        : Generic::Homing(config) {
    homingSemaphoreHandle_ = xSemaphoreCreateBinary();
    if (homingSemaphoreHandle_ != nullptr) {
        xSemaphoreGive(homingSemaphoreHandle_);
    }
}

Homing::~Homing() {
    abortHoming();
    if (homingSemaphoreHandle_ != nullptr) {
        vSemaphoreDelete(homingSemaphoreHandle_);
    }
}

bool Homing::homeAsync(Generic::Axis& axis, const Config& config, HomingCallback callback) {
    if (isHoming() || homingTaskHandle_ != nullptr) {
        LOG_WARN("Homing is already in progress");
        return false;
    }

    if (isDualEndstopStrategy(config.strategy)) {
        if (!axis.hasEndstopMin() || !axis.hasEndstopMax()) {
            LOG_WARN("Cannot start async dual-endstop homing: Both Min and Max endstops must be attached");
            return false;
        }
    } else {
        Generic::Axis::EndstopPosition targetEndstop = (config.strategy == Strategy::Min)
                                                        ? Generic::Axis::EndstopPosition::Min
                                                        : Generic::Axis::EndstopPosition::Max;
        if (!axis.hasEndstop(targetEndstop)) {
            LOG_WARN("Cannot start async homing: Target endstop not attached");
            return false;
        }
    }

    activeAxis_ = &axis;
    homingCallback_ = std::move(callback);
    pendingConfig_ = config;
    lastResult_ = false;

    // Reset completion semaphore before starting
    if (homingSemaphoreHandle_ != nullptr) {
        xSemaphoreTake(homingSemaphoreHandle_, 0);
    }

    BaseType_t res = xTaskCreatePinnedToCore(
        Homing::homingTask,       /* Task function */
        "HomingTask",             /* Task name (max. 16 characters by default) */
        4096,                     /* Stack size in bytes */
        this,                     /* Parameter passed as input of the task */
        configMAX_PRIORITIES - 3, /* Task priority */
        &homingTaskHandle_,       /* Task handle */
        1                         /* CPU core to use */
    );

    if (res != pdPASS) {
        LOG_ERROR("Failed to create FreeRTOS homing task");
        activeAxis_ = nullptr;
        if (homingSemaphoreHandle_ != nullptr) {
            xSemaphoreGive(homingSemaphoreHandle_);
        }
        return false;
    }

    return true;
}

bool Homing::homeAsync(Generic::Axis& axis, Strategy strategy, HomingCallback callback) {
    Config cfg = config_;
    cfg.strategy = strategy;
    return homeAsync(axis, cfg, std::move(callback));
}

bool Homing::homeAsync(Generic::Axis& axis, HomingCallback callback) {
    return homeAsync(axis, config_, std::move(callback));
}

bool Homing::homeMinMaxAsync(Generic::Axis& axis, HomingCallback callback) {
    return homeMinMaxAsync(axis, config_, std::move(callback));
}

bool Homing::homeMinMaxAsync(Generic::Axis& axis, const Config& config, HomingCallback callback) {
    Config cfg = config;
    cfg.strategy = Strategy::MinThenMax;
    return homeAsync(axis, cfg, std::move(callback));
}

bool Homing::homeMaxMinAsync(Generic::Axis& axis, HomingCallback callback) {
    return homeMaxMinAsync(axis, config_, std::move(callback));
}

bool Homing::homeMaxMinAsync(Generic::Axis& axis, const Config& config, HomingCallback callback) {
    Config cfg = config;
    cfg.strategy = Strategy::MaxThenMin;
    return homeAsync(axis, cfg, std::move(callback));
}

bool Homing::waitForHoming(uint32_t timeoutMs) {
    if (!isHoming() && homingTaskHandle_ == nullptr) {
        return lastResult_;
    }

    if (homingSemaphoreHandle_ == nullptr) {
        return false;
    }

    TickType_t ticks = (timeoutMs == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    if (xSemaphoreTake(homingSemaphoreHandle_, ticks) == pdTRUE) {
        xSemaphoreGive(homingSemaphoreHandle_);
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
            auto cleanupEndstop = [&](Generic::Axis::EndstopPosition pos) {
                Generic::Endstop* e = activeAxis_->getEndstop(pos);
                if (e != nullptr) {
                    e->unregisterCallback();
                }
                restoreLimitProtection(*activeAxis_, pos);
            };
            if (isDualEndstopStrategy(pendingConfig_.strategy)) {
                cleanupEndstop(Generic::Axis::EndstopPosition::Min);
                cleanupEndstop(Generic::Axis::EndstopPosition::Max);
            } else {
                Generic::Axis::EndstopPosition target = (pendingConfig_.strategy == Strategy::Min)
                                                        ? Generic::Axis::EndstopPosition::Min
                                                        : Generic::Axis::EndstopPosition::Max;
                cleanupEndstop(target);
            }
            activeAxis_->setHoming(false);
            activeAxis_ = nullptr;
        }

        state_ = State::Failed;
        lastResult_ = false;

        if (homingCallback_) {
            auto cb = std::move(homingCallback_);
            cb(false);
        }

        if (homingSemaphoreHandle_ != nullptr) {
            xSemaphoreGive(homingSemaphoreHandle_);
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

    if (self->homingSemaphoreHandle_ != nullptr) {
        xSemaphoreGive(self->homingSemaphoreHandle_);
    }

    self->homingTaskHandle_ = nullptr;
    LOG_INFO("FreeRTOS homing task completed (%s)", success ? "SUCCESS" : "FAILED");
    vTaskDelete(NULL);
}

} // namespace ESP32
} // namespace Stepper
