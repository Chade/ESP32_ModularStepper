#include "StopController.hpp"

#include <algorithm>

#include "Log.hpp"

namespace Stepper {
namespace Generic {

IStoppable* StopController::findTargetLocked(int16_t id) const {
    for (const auto& entry : targets_) {
        if (entry.id == id) {
            return entry.target;
        }
    }
    return nullptr;
}

bool StopController::registerTarget(int16_t id, IStoppable& target) {
    if (id < 0) {
        LOG_ERROR("StopController: invalid target id %d (must be >= 0)", static_cast<int>(id));
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (findTargetLocked(id) != nullptr) {
        LOG_ERROR("StopController: duplicate target id %d", static_cast<int>(id));
        return false;
    }

    targets_.push_back({id, &target});
    return true;
}

bool StopController::unregisterTarget(int16_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::remove_if(targets_.begin(), targets_.end(),
                             [id](const Entry& e) { return e.id == id; });
    if (it == targets_.end()) {
        return false;
    }
    targets_.erase(it, targets_.end());
    return true;
}

bool StopController::unregisterTarget(const IStoppable& target) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::remove_if(targets_.begin(), targets_.end(),
                             [&target](const Entry& e) { return e.target == &target; });
    if (it == targets_.end()) {
        return false;
    }
    targets_.erase(it, targets_.end());
    return true;
}

bool StopController::hasTarget(int16_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return findTargetLocked(id) != nullptr;
}

size_t StopController::targetCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return targets_.size();
}

bool StopController::pause(int16_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    IStoppable* target = findTargetLocked(id);
    if (target == nullptr) {
        LOG_WARN("StopController: target id %d not found for pause", static_cast<int>(id));
        return false;
    }
    return target->pause();
}

bool StopController::resume(int16_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    IStoppable* target = findTargetLocked(id);
    if (target == nullptr) {
        LOG_WARN("StopController: target id %d not found for resume", static_cast<int>(id));
        return false;
    }
    return target->resume();
}

bool StopController::fastStop(int16_t id, float deceleration) {
    std::lock_guard<std::mutex> lock(mutex_);
    IStoppable* target = findTargetLocked(id);
    if (target == nullptr) {
        LOG_WARN("StopController: target id %d not found for fastStop", static_cast<int>(id));
        return false;
    }
    if (deceleration > 0.0f) {
        target->fastStop(deceleration);
    } else {
        target->fastStop();
    }
    return true;
}

bool StopController::emergencyStop(int16_t id, bool disableDriver) {
    std::lock_guard<std::mutex> lock(mutex_);
    IStoppable* target = findTargetLocked(id);
    if (target == nullptr) {
        LOG_WARN("StopController: target id %d not found for emergencyStop", static_cast<int>(id));
        return false;
    }
    target->emergencyStop(disableDriver);
    return true;
}

bool StopController::clearEmergencyStop(int16_t id, bool enableDriver) {
    std::lock_guard<std::mutex> lock(mutex_);
    IStoppable* target = findTargetLocked(id);
    if (target == nullptr) {
        LOG_WARN("StopController: target id %d not found for clearEmergencyStop", static_cast<int>(id));
        return false;
    }
    return target->clearEmergencyStop(enableDriver);
}

void StopController::pauseAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : targets_) {
        entry.target->pause();
    }
}

void StopController::resumeAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : targets_) {
        entry.target->resume();
    }
}

void StopController::fastStopAll(float deceleration) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : targets_) {
        if (deceleration > 0.0f) {
            entry.target->fastStop(deceleration);
        } else {
            entry.target->fastStop();
        }
    }
}

void StopController::emergencyStopAll(bool disableDrivers) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : targets_) {
        entry.target->emergencyStop(disableDrivers);
    }
}

void StopController::clearEmergencyStopAll(bool enableDrivers) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : targets_) {
        entry.target->clearEmergencyStop(enableDrivers);
    }
}

} // namespace Generic
} // namespace Stepper
