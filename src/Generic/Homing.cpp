#include "Homing.hpp"
#include "Log.hpp"

#if defined(ARDUINO) || defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#include <chrono>
#include <thread>
#endif

namespace Stepper {
namespace Generic {

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

    // Determine approach and retract directions (Min = -1 -> CW/Backward, Max = +1 -> CCW/Forward)
    Direction searchDir = config.direction;
    if (searchDir == Direction::Neutral) {
        searchDir = (config.endstop == Axis::EndstopPosition::Min)
                    ? Direction::Clockwise
                    : Direction::Counterclockwise;
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

    float retractVel = (config.slowVelocity > 0.0f) ? config.slowVelocity : config.fastVelocity;

    // Step 0: Initial retract if already triggered before homing
    if (targetEndstop->isTriggered()) {
        LOG_INFO("Endstop already triggered prior to homing, retracting (%.2f)...", config.retractDistance);
        if (config.retractDistance <= 0.0f) {
            LOG_ERROR("Cannot retract: retractDistance must be > 0");
            cleanupOnFailure();
            return false;
        }
        state_ = State::Retracting;
        axis.run(config.retractDistance, retractVel, config.acceleration, config.deceleration, retractDir);

        while (axis.getState() != Stepper::State::Stopped) {
            yieldMs(1);
        }
        axis.stop();
        yieldMs(20);

        if (targetEndstop->isTriggered()) {
            LOG_ERROR("Endstop still triggered after initial retract distance (%.2f)", config.retractDistance);
            cleanupOnFailure();
            return false;
        }
    }

    // Phase 1: Fast Approach (bounded by maxTravelDistance if > 0)
    LOG_INFO("Homing Phase 1: Fast approach...");
    state_ = State::FastApproach;
    endstopTriggered = false;
    if (config.maxTravelDistance > 0.0f) {
        axis.run(config.maxTravelDistance, config.fastVelocity, config.acceleration, config.deceleration, searchDir);
    } else {
        axis.run(config.fastVelocity, config.acceleration, config.deceleration, searchDir);
    }

    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (axis.getState() == Stepper::State::Stopped) {
            if (!endstopTriggered && !targetEndstop->isTriggered()) {
                LOG_ERROR("Homing fast approach exceeded max travel distance (%.2f)", config.maxTravelDistance);
                cleanupOnFailure();
                return false;
            }
            break;
        }
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    // Phase 2: Retract / Back-off
    LOG_INFO("Homing Phase 2: Retracting (%.2f)...", config.retractDistance);
    if (config.retractDistance > 0.0f) {
        state_ = State::Retracting;
        axis.run(config.retractDistance, retractVel, config.acceleration, config.deceleration, retractDir);

        while (axis.getState() != Stepper::State::Stopped) {
            yieldMs(1);
        }
        axis.stop();
        yieldMs(20);

        if (targetEndstop->isTriggered()) {
            LOG_ERROR("Endstop still triggered after retract distance (%.2f)", config.retractDistance);
            cleanupOnFailure();
            return false;
        }
    }

    // Phase 3: Slow Approach (Latch), bounded by 2x retractDistance (or maxTravelDistance)
    LOG_INFO("Homing Phase 3: Slow latch approach...");
    state_ = State::SlowApproach;
    endstopTriggered = false;
    float slowMaxDistance = (config.retractDistance > 0.0f) ? (config.retractDistance * 2.0f) : config.maxTravelDistance;
    if (slowMaxDistance > 0.0f) {
        axis.run(slowMaxDistance, config.slowVelocity, config.acceleration, config.deceleration, searchDir);
    } else {
        axis.run(config.slowVelocity, config.acceleration, config.deceleration, searchDir);
    }

    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (axis.getState() == Stepper::State::Stopped) {
            if (!endstopTriggered && !targetEndstop->isTriggered()) {
                LOG_ERROR("Homing slow latch exceeded max distance (%.2f)", slowMaxDistance);
                cleanupOnFailure();
                return false;
            }
            break;
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

        while (axis.getState() != Stepper::State::Stopped) {
            yieldMs(1);
        }
        axis.stop();
        yieldMs(20);

        if (targetEndstop->isTriggered()) {
            LOG_ERROR("Endstop still triggered after final back-off distance (%.2f)", config.backoffDistance);
            cleanupOnFailure();
            return false;
        }
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
