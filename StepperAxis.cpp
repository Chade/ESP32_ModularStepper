#include "StepperAxis.hpp"
#include <thread>

#if defined(ARDUINO) || defined(ESP32)
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace Stepper {
namespace Generic {

Axis::Axis(Generic::Motor& motor)
    : motor_(motor) {
}

void Axis::setEndstopMin(Generic::Endstop* endstop, bool enableLimitProtection) {
    endstopMin_ = endstop;
    if (enableLimitProtection) {
        enableLimitProtectionMin(true);
    }
}

void Axis::setEndstopMin(Generic::Endstop& endstop, bool enableLimitProtection) {
    setEndstopMin(&endstop, enableLimitProtection);
}

void Axis::setEndstopMax(Generic::Endstop* endstop, bool enableLimitProtection) {
    endstopMax_ = endstop;
    if (enableLimitProtection) {
        enableLimitProtectionMax(true);
    }
}

void Axis::setEndstopMax(Generic::Endstop& endstop, bool enableLimitProtection) {
    setEndstopMax(&endstop, enableLimitProtection);
}

void Axis::setEndstops(Generic::Endstop* minEndstop, Generic::Endstop* maxEndstop, bool enableLimitProtectionMin, bool enableLimitProtectionMax) {
    setEndstopMin(minEndstop, enableLimitProtectionMin);
    setEndstopMax(maxEndstop, enableLimitProtectionMax);
}

void Axis::setEndstop(EndstopPosition pos, Generic::Endstop* endstop, bool enableLimitProtection) {
    if (pos == EndstopPosition::Min) {
        setEndstopMin(endstop, enableLimitProtection);
    } else if (pos == EndstopPosition::Max) {
        setEndstopMax(endstop, enableLimitProtection);
    }
}

void Axis::stop() {
    motor_.stop();
}

void Axis::emergencyStop() {
    motor_.stop();
    isHoming_ = false;
    homingState_ = HomingState::Failed;
}

void Axis::enableLimitProtectionMin(bool enable) {
    limitProtectionMinEnabled_ = enable;
    if (enable && endstopMin_) {
        endstopMin_->registerCallback(Generic::Endstop::Edge::Both, [this](Generic::Endstop::Edge) {
            if (limitProtectionMinEnabled_ && !isHoming_ && endstopMin_->isTriggered()) {
                LOG_WARN("Min limit switch triggered! Halting motion.");
                stop();
            }
        });
    }
}

void Axis::enableLimitProtectionMax(bool enable) {
    limitProtectionMaxEnabled_ = enable;
    if (enable && endstopMax_) {
        endstopMax_->registerCallback(Generic::Endstop::Edge::Both, [this](Generic::Endstop::Edge) {
            if (limitProtectionMaxEnabled_ && !isHoming_ && endstopMax_->isTriggered()) {
                LOG_WARN("Max limit switch triggered! Halting motion.");
                stop();
            }
        });
    }
}

uint32_t Axis::getTimeMs() const {
    if (timeFn_) {
        return timeFn_();
    }
#if defined(ARDUINO) || defined(ESP32)
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
#else
    using namespace std::chrono;
    static auto epoch = steady_clock::now();
    return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - epoch).count());
#endif
}

void Axis::yieldMs(uint32_t delayMs) const {
    if (yieldFn_) {
        yieldFn_(delayMs);
        return;
    }
#if defined(ARDUINO) || defined(ESP32)
    vTaskDelay(pdMS_TO_TICKS(delayMs > 0 ? delayMs : 1));
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs > 0 ? delayMs : 1));
#endif
}

bool Axis::home(const HomingConfig& config) {
    Generic::Endstop* targetEndstop = getEndstop(config.endstop);
    if (!targetEndstop) {
        LOG_WARN("Cannot home: Endstop %s not configured", 
                 (config.endstop == EndstopPosition::Min) ? "Min" : "Max");
        homingState_ = HomingState::Failed;
        return false;
    }

    isHoming_ = true;
    isHomed_ = false;

    // Determine approach direction
    Direction searchDir = config.direction;
    if (searchDir == Direction::Neutral) {
        searchDir = (config.endstop == EndstopPosition::Min)
                    ? Direction::Counterclockwise
                    : Direction::Clockwise;
    }

    Direction retractDir = (searchDir == Direction::Clockwise)
                           ? Direction::Counterclockwise
                           : Direction::Clockwise;

    volatile bool endstopTriggered = false;
    targetEndstop->registerCallback(Generic::Endstop::Edge::Both, [&endstopTriggered, targetEndstop, this](Generic::Endstop::Edge) {
        if (targetEndstop->isTriggered()) {
            endstopTriggered = true;
            this->stop();
        }
    });

    auto startOverallTime = getTimeMs();
    auto isTimeout = [&](uint32_t phaseStart) {
        if (config.timeoutMs == 0) return false;
        return ((getTimeMs() - phaseStart) >= config.timeoutMs) || 
               ((getTimeMs() - startOverallTime) >= (config.timeoutMs * 2));
    };

    // Step 0: Initial retract if already triggered
    if (targetEndstop->isTriggered()) {
        LOG_INFO("Endstop already triggered prior to homing, retracting first...");
        homingState_ = HomingState::Retracting;
        float retractVel = (config.slowVelocity > 0.0f) ? config.slowVelocity : config.fastVelocity;
        runDistance(config.retractDistance, retractVel, config.acceleration, config.deceleration, retractDir);

        uint32_t retractStart = getTimeMs();
        while (targetEndstop->isTriggered()) {
            if (isTimeout(retractStart)) {
                LOG_ERROR("Initial retract timed out waiting for endstop to release");
                stop();
                targetEndstop->unregisterCallback();
                homingState_ = HomingState::Failed;
                isHoming_ = false;
                return false;
            }
            yieldMs(1);
        }
        stop();
        yieldMs(20);
    }

    // Phase 1: Fast Approach
    LOG_INFO("Homing Phase 1: Fast approach...");
    homingState_ = HomingState::FastApproach;
    endstopTriggered = false;
    runVelocity(config.fastVelocity, config.acceleration, config.deceleration, searchDir);

    uint32_t phase1Start = getTimeMs();
    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (isTimeout(phase1Start)) {
            LOG_ERROR("Homing fast approach timed out");
            stop();
            targetEndstop->unregisterCallback();
            homingState_ = HomingState::Failed;
            isHoming_ = false;
            return false;
        }
        yieldMs(1);
    }
    stop();
    yieldMs(20);

    // Phase 2: Retract / Back-off
    LOG_INFO("Homing Phase 2: Retracting...");
    homingState_ = HomingState::Retracting;
    float retractVel = (config.slowVelocity > 0.0f) ? config.slowVelocity : config.fastVelocity;
    runDistance(config.retractDistance, retractVel, config.acceleration, config.deceleration, retractDir);

    uint32_t phase2Start = getTimeMs();
    while (targetEndstop->isTriggered()) {
        if (isTimeout(phase2Start)) {
            LOG_ERROR("Homing retract timed out waiting for endstop release");
            stop();
            targetEndstop->unregisterCallback();
            homingState_ = HomingState::Failed;
            isHoming_ = false;
            return false;
        }
        yieldMs(1);
    }
    // Allow small settle time
    yieldMs(30);
    stop();

    // Phase 3: Slow Approach (Latch)
    LOG_INFO("Homing Phase 3: Slow latch approach...");
    homingState_ = HomingState::SlowApproach;
    endstopTriggered = false;
    runVelocity(config.slowVelocity, config.acceleration, config.deceleration, searchDir);

    uint32_t phase3Start = getTimeMs();
    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (isTimeout(phase3Start)) {
            LOG_ERROR("Homing slow latch approach timed out");
            stop();
            targetEndstop->unregisterCallback();
            homingState_ = HomingState::Failed;
            isHoming_ = false;
            return false;
        }
        yieldMs(1);
    }
    stop();
    targetEndstop->unregisterCallback();

    // Phase 4: Done
    setPosition(config.homePosition);
    isHomed_ = true;
    isHoming_ = false;
    homingState_ = HomingState::Done;
    LOG_INFO("Homing successful. Position set to %.2f", config.homePosition);
    return true;
}

bool Axis::home(EndstopPosition pos) {
    HomingConfig cfg = homingConfig_;
    cfg.endstop = pos;
    return home(cfg);
}

bool Axis::homeMin() {
    return home(EndstopPosition::Min);
}

bool Axis::homeMax() {
    return home(EndstopPosition::Max);
}

} // namespace Generic
} // namespace Stepper

