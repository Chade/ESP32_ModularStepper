#include "Homing.hpp"
#include "Log.hpp"

#include <chrono>
#include <thread>

#if defined(ARDUINO) || defined(ESP32)
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace Stepper {
namespace Generic {

uint32_t Homing::getTimeMs() const {
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

void Homing::yieldMs(uint32_t delayMs) const {
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

void Homing::restoreLimitProtection(Axis& axis, Axis::EndstopPosition pos) {
    if (pos == Axis::EndstopPosition::Min && axis.isLimitProtectionMinEnabled()) {
        axis.enableLimitProtectionMin(true);
    } else if (pos == Axis::EndstopPosition::Max && axis.isLimitProtectionMaxEnabled()) {
        axis.enableLimitProtectionMax(true);
    }
}

bool Homing::home(Axis& axis, const Config& config) {
    Generic::Endstop* targetEndstop = axis.getEndstop(config.endstop);
    if (!targetEndstop) {
        LOG_WARN("Cannot home: Endstop %s not configured",
                 (config.endstop == Axis::EndstopPosition::Min) ? "Min" : "Max");
        state_ = State::Failed;
        return false;
    }

    axis.setHoming(true);
    axis.setHomed(false);

    // Determine approach direction
    Direction searchDir = config.direction;
    if (searchDir == Direction::Neutral) {
        searchDir = (config.endstop == Axis::EndstopPosition::Min)
                    ? Direction::Counterclockwise
                    : Direction::Clockwise;
    }

    Direction retractDir = (searchDir == Direction::Clockwise)
                           ? Direction::Counterclockwise
                           : Direction::Clockwise;

    volatile bool endstopTriggered = false;
    targetEndstop->registerCallback(Generic::Endstop::Edge::Both, [&endstopTriggered, targetEndstop, &axis, this](Generic::Endstop::Edge) {
        if ((state_ == State::FastApproach || state_ == State::SlowApproach) && targetEndstop->isTriggered()) {
            endstopTriggered = true;
            axis.stop();
        }
    });

    auto cleanupOnFailure = [&]() {
        axis.stop();
        targetEndstop->unregisterCallback();
        restoreLimitProtection(axis, config.endstop);
        state_ = State::Failed;
        axis.setHoming(false);
    };

    auto startOverallTime = getTimeMs();
    auto isTimeout = [&](uint32_t phaseStart) {
        if (config.timeoutMs == 0) return false;
        return ((getTimeMs() - phaseStart) >= config.timeoutMs) ||
               ((getTimeMs() - startOverallTime) >= (config.timeoutMs * 2));
    };

    // Step 0: Initial retract if already triggered
    if (targetEndstop->isTriggered()) {
        LOG_INFO("Endstop already triggered prior to homing, retracting first...");
        state_ = State::Retracting;
        float retractVel = (config.slowVelocity > 0.0f) ? config.slowVelocity : config.fastVelocity;
        axis.run(config.retractDistance, retractVel, config.acceleration, config.deceleration, retractDir);

        bool continuousRetract = false;
        uint32_t retractStart = getTimeMs();
        while (true) {
            if (isTimeout(retractStart)) {
                LOG_ERROR("Initial retract timed out waiting for endstop to release");
                cleanupOnFailure();
                return false;
            }
            bool triggered = targetEndstop->isTriggered();
            if (continuousRetract) {
                if (!triggered) {
                    break;
                }
            } else if (axis.getState() == Stepper::State::Stopped) {
                if (!triggered) {
                    break;
                }
                continuousRetract = true;
                axis.run(retractVel, config.acceleration, config.deceleration, retractDir);
            }
            yieldMs(1);
        }
        axis.stop();
        yieldMs(20);
    }

    // Phase 1: Fast Approach
    LOG_INFO("Homing Phase 1: Fast approach...");
    state_ = State::FastApproach;
    endstopTriggered = false;
    axis.run(config.fastVelocity, config.acceleration, config.deceleration, searchDir);

    uint32_t phase1Start = getTimeMs();
    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (isTimeout(phase1Start)) {
            LOG_ERROR("Homing fast approach timed out");
            cleanupOnFailure();
            return false;
        }
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    // Phase 2: Retract / Back-off
    LOG_INFO("Homing Phase 2: Retracting...");
    state_ = State::Retracting;
    float retractVel = (config.slowVelocity > 0.0f) ? config.slowVelocity : config.fastVelocity;
    axis.run(config.retractDistance, retractVel, config.acceleration, config.deceleration, retractDir);

    bool continuousRetract = false;
    uint32_t phase2Start = getTimeMs();
    while (true) {
        if (isTimeout(phase2Start)) {
            LOG_ERROR("Homing retract timed out");
            cleanupOnFailure();
            return false;
        }
        bool triggered = targetEndstop->isTriggered();
        if (continuousRetract) {
            // In continuous back-off mode: stop as soon as endstop releases
            if (!triggered) {
                break;
            }
        } else if (axis.getState() == Stepper::State::Stopped) {
            if (!triggered) {
                break; // Retract distance completed and endstop is clear
            }
            // Endstop still triggered after retractDistance: continue backing off until released
            continuousRetract = true;
            axis.run(retractVel, config.acceleration, config.deceleration, retractDir);
        }
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    // Phase 3: Slow Approach (Latch)
    LOG_INFO("Homing Phase 3: Slow latch approach...");
    state_ = State::SlowApproach;
    endstopTriggered = false;
    axis.run(config.slowVelocity, config.acceleration, config.deceleration, searchDir);

    uint32_t phase3Start = getTimeMs();
    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (isTimeout(phase3Start)) {
            LOG_ERROR("Homing slow latch approach timed out");
            cleanupOnFailure();
            return false;
        }
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    // Assign home reference position at the exact slow-latch switch trigger point
    axis.setPosition(config.homePosition);

    // Phase 4: Optional final back-off so the axis does not rest on the switch
    if (config.backoffDistance > 0.0f) {
        LOG_INFO("Homing Phase 4: Backing off switch (%.2f)...", config.backoffDistance);
        state_ = State::BackingOff;
        axis.run(config.backoffDistance, retractVel, config.acceleration, config.deceleration, retractDir);

        bool continuousBackoff = false;
        uint32_t phase4Start = getTimeMs();
        while (true) {
            if (isTimeout(phase4Start)) {
                LOG_ERROR("Homing final back-off timed out");
                cleanupOnFailure();
                return false;
            }
            bool triggered = targetEndstop->isTriggered();
            if (continuousBackoff) {
                if (!triggered) {
                    break;
                }
            } else if (axis.getState() == Stepper::State::Stopped) {
                if (!triggered) {
                    break;
                }
                continuousBackoff = true;
                axis.run(retractVel, config.acceleration, config.deceleration, retractDir);
            }
            yieldMs(1);
        }
        axis.stop();
        yieldMs(20);
    }

    targetEndstop->unregisterCallback();
    restoreLimitProtection(axis, config.endstop);

    // Phase 5: Done
    axis.setHomed(true);
    axis.setHoming(false);
    state_ = State::Done;
    LOG_INFO("Homing successful. Position: %.2f (home ref: %.2f)", axis.getPosition(), config.homePosition);
    return true;
}

bool Homing::home(Axis& axis, Axis::EndstopPosition pos) {
    Config cfg = config_;
    cfg.endstop = pos;
    return home(axis, cfg);
}

bool Homing::homeMin(Axis& axis) {
    return home(axis, Axis::EndstopPosition::Min);
}

bool Homing::homeMax(Axis& axis) {
    return home(axis, Axis::EndstopPosition::Max);
}

} // namespace Generic
} // namespace Stepper
