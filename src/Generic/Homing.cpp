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

void Homing::finalizeAxisState(Axis& axis) {
    auto& generator = axis.motor().generator();
    generator.driver().reset(true);
    generator.resetState(false); // preserve calibrated positionSteps

    axis.setHomed(true);
    axis.setHoming(false);
    state_ = State::Done;
}

bool Homing::homeToEndstop(Axis& axis, Axis::EndstopPosition endstopPos, const Config& config) {
    Generic::Endstop* targetEndstop = axis.getEndstop(endstopPos);
    if (!targetEndstop) {
        LOG_WARN("Cannot home: Endstop %s not configured",
                 (endstopPos == Axis::EndstopPosition::Min) ? "Min" : "Max");
        return false;
    }
    
    volatile bool endstopTriggered = false;
    targetEndstop->registerCallback(Generic::Endstop::Edge::Both, [&endstopTriggered, targetEndstop, &axis, this](Generic::Endstop::Edge) {
        if ((state_ == State::FastApproach || state_ == State::SlowApproach) && targetEndstop->isTriggered()) {
            endstopTriggered = true;
            axis.stop();
        }
    });
    
    auto failEndstop = [&]() {
        axis.stop();
        targetEndstop->unregisterCallback();
        restoreLimitProtection(axis, endstopPos);
        return false;
    };
    
    Direction searchDir = searchDirectionFor(endstopPos);
    Direction retractDir = reverseDirection(searchDir);
    float searchVelFast = fabs(config.fastVelocity) * sign(cast_enum_to_base(searchDir));
    float searchVelSlow = fabs(config.slowVelocity) * sign(cast_enum_to_base(searchDir));
    float retractVel = fabs(config.slowVelocity) * sign(cast_enum_to_base(retractDir));
    
    // Step 0: Initial retract if already triggered before approaching
    if (targetEndstop->isTriggered()) {
        LOG_INFO("Endstop already triggered prior to homing, retracting (%.2f)...", config.retractDistance);
        if (config.retractDistance <= 0.0f) {
            LOG_ERROR("Cannot retract: retractDistance must be > 0");
            return failEndstop();
        }
        state_ = State::Retracting;
        axis.run(config.retractDistance, retractVel, config.acceleration, config.deceleration);

        while (axis.getState() != Stepper::State::Stopped) {
            yieldMs(1);
        }
        axis.stop();
        yieldMs(20);

        if (targetEndstop->isTriggered()) {
            LOG_ERROR("Endstop still triggered after initial retract distance (%.2f)", config.retractDistance);
            return failEndstop();
        }
    }

    // Phase 1: Fast Approach (bounded by maxTravelDistance if > 0)
    LOG_INFO("Homing Phase 1: Fast approach...");
    state_ = State::FastApproach;
    endstopTriggered = false;
    if (config.maxTravelDistance > 0.0f) {
        axis.run(config.maxTravelDistance, searchVelFast, config.acceleration, config.deceleration);
    } else {
        axis.run(searchVelFast, config.acceleration, config.deceleration);
    }

    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (axis.getState() == Stepper::State::Stopped) {
            if (!endstopTriggered && !targetEndstop->isTriggered()) {
                LOG_ERROR("Homing fast approach exceeded max travel distance (%.2f)", config.maxTravelDistance);
                return failEndstop();
            }
            break;
        }
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    // Phase 2: Retract / Back-off
    LOG_INFO("Homing Phase 2: Retracting (%.2f)...", config.retractDistance);
    if (config.retractDistance != 0.0f) {
        state_ = State::Retracting;
        axis.run(config.retractDistance, retractVel, config.acceleration, config.deceleration);

        while (axis.getState() != Stepper::State::Stopped) {
            yieldMs(1);
        }
        axis.stop();
        yieldMs(20);

        if (targetEndstop->isTriggered()) {
            LOG_ERROR("Endstop still triggered after retract distance (%.2f)", config.retractDistance);
            return failEndstop();
        }
    }

    // Phase 3: Slow Approach (Latch), bounded by 2x retractDistance (or maxTravelDistance)
    LOG_INFO("Homing Phase 3: Slow latch approach...");
    state_ = State::SlowApproach;
    endstopTriggered = false;
    float slowMaxDistance = (config.retractDistance > 0.0f) ? (config.retractDistance * 2.0f) : config.maxTravelDistance;
    if (slowMaxDistance > 0.0f) {
        axis.run(slowMaxDistance, searchVelSlow, config.acceleration, config.deceleration);
    } else {
        axis.run(searchVelSlow, config.acceleration, config.deceleration);
    }

    while (!endstopTriggered && !targetEndstop->isTriggered()) {
        if (axis.getState() == Stepper::State::Stopped) {
            if (!endstopTriggered && !targetEndstop->isTriggered()) {
                LOG_ERROR("Homing slow latch exceeded max distance (%.2f)", slowMaxDistance);
                return failEndstop();
            }
            break;
        }
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    targetEndstop->unregisterCallback();
    restoreLimitProtection(axis, endstopPos);
    return true;
}

bool Homing::backoffFromEndstop(Axis& axis, Axis::EndstopPosition endstopPos, const Config& config) {
    if (config.backoffDistance <= 0.0f) {
        return true;
    }

    Generic::Endstop* targetEndstop = axis.getEndstop(endstopPos);
    Direction retractDir = reverseDirection(searchDirectionFor(endstopPos));
    float retractVel = fabs(config.slowVelocity) * sign(cast_enum_to_base(retractDir));

    LOG_INFO("Homing Phase 4: Backing off switch (%.2f)...", config.backoffDistance);
    state_ = State::BackingOff;
    axis.run(config.backoffDistance, retractVel, config.acceleration, config.deceleration);

    while (axis.getState() != Stepper::State::Stopped) {
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    if (targetEndstop && targetEndstop->isTriggered()) {
        LOG_ERROR("Endstop still triggered after final back-off distance (%.2f)", config.backoffDistance);
        return false;
    }
    return true;
}

bool Homing::backoffFromEndstop(Axis& axis, Axis::EndstopPosition endstopPos, float distance, float velocity, float acceleration, float dceleration) {
    if (distance == 0.0f) {
        LOG_ERROR("Backoff distance must not be zero");
        return false;
    }

    Generic::Endstop* targetEndstop = axis.getEndstop(endstopPos);

    if (targetEndstop == nullptr) {
        LOG_ERROR("No endstop at position %d", cast_enum_to_base(endstopPos));
        return false;
    }

    if (!targetEndstop->isTriggered()) {
        return true;
    }

    Direction backoffDirection = reverseDirection(searchDirectionFor(endstopPos));
    float signedVelocity = fabs(velocity) * sign(cast_enum_to_base(backoffDirection));
    axis.run(distance, signedVelocity, acceleration, dceleration);

    while (axis.getState() != Stepper::State::Stopped) {
        yieldMs(1);
    }
    axis.stop();
    yieldMs(20);

    if (targetEndstop->isTriggered()) {
        LOG_ERROR("Endstop still triggered after backing off (%.2f)", distance);
        return false;
    }

    return true;
}


bool Homing::home(Axis& axis, const Config& config) {
    if (isDualEndstopStrategy(config.strategy)) {
        if (!axis.hasEndstopMin() || !axis.hasEndstopMax()) {
            LOG_ERROR("Cannot perform dual-endstop homing: Both Min and Max endstops must be attached");
            state_ = State::Failed;
            return false;
        }

        const bool minFirst = (config.strategy == Strategy::MinThenMax);
        const Axis::EndstopPosition firstEndstop  = minFirst ? Axis::EndstopPosition::Min : Axis::EndstopPosition::Max;
        const Axis::EndstopPosition secondEndstop = minFirst ? Axis::EndstopPosition::Max : Axis::EndstopPosition::Min;

        axis.setHoming(true);
        axis.setHomed(false);

        // 1. Home to first endstop and establish reference position
        LOG_INFO("Dual-endstop homing (1/2): Homing to %s endstop...", minFirst ? "Min" : "Max");
        if (!homeToEndstop(axis, firstEndstop, config)) {
            state_ = State::Failed;
            axis.setHoming(false);
            return false;
        }
        axis.setPosition(config.homePosition);

        // 2. Home to second endstop and calibrate axis travel limits
        LOG_INFO("Dual-endstop homing (2/2): Homing to %s endstop...", minFirst ? "Max" : "Min");
        if (!homeToEndstop(axis, secondEndstop, config)) {
            state_ = State::Failed;
            axis.setHoming(false);
            return false;
        }
        float measuredSecond = axis.getPosition();
        float minPos = minFirst ? config.homePosition : measuredSecond;
        float maxPos = minFirst ? measuredSecond : config.homePosition;
        if (maxPos <= minPos) {
            LOG_ERROR("Invalid calibrated limits [%.2f, %.2f]", minPos, maxPos);
            state_ = State::Failed;
            axis.setHoming(false);
            return false;
        }
        axis.setLimits(minPos, maxPos);
        LOG_INFO("Axis limits calibrated to [%.2f, %.2f]", minPos, maxPos);

        // 3. Optional final back-off off the second switch
        if (!backoffFromEndstop(axis, secondEndstop, config)) {
            state_ = State::Failed;
            axis.setHoming(false);
            return false;
        }

        finalizeAxisState(axis);
        LOG_INFO("Dual-endstop homing successful. Position: %.2f, Limits: [%.2f, %.2f]",
                 axis.getPosition(), axis.getMinLimit(), axis.getMaxLimit());
        return true;
    }

    // Single-endstop strategy (Strategy::Min or Strategy::Max)
    const Axis::EndstopPosition targetEndstop = (config.strategy == Strategy::Min)
                                                ? Axis::EndstopPosition::Min
                                                : Axis::EndstopPosition::Max;
    if (!axis.hasEndstop(targetEndstop)) {
        LOG_WARN("Cannot home: Endstop %s not configured",
                 (targetEndstop == Axis::EndstopPosition::Min) ? "Min" : "Max");
        state_ = State::Failed;
        return false;
    }

    axis.setHoming(true);
    axis.setHomed(false);

    if (!homeToEndstop(axis, targetEndstop, config)) {
        state_ = State::Failed;
        axis.setHoming(false);
        return false;
    }

    // Assign home reference position at the exact slow-latch switch trigger point
    axis.setPosition(config.homePosition);

    if (!backoffFromEndstop(axis, targetEndstop, config)) {
        state_ = State::Failed;
        axis.setHoming(false);
        return false;
    }

    finalizeAxisState(axis);
    LOG_INFO("Homing successful. Position: %.2f (home ref: %.2f)", axis.getPosition(), config.homePosition);
    return true;
}

bool Homing::home(Axis& axis) {
    return home(axis, config_);
}

bool Homing::home(Axis& axis, Strategy strategy) {
    Config cfg = config_;
    cfg.strategy = strategy;
    return home(axis, cfg);
}

bool Homing::homeMin(Axis& axis) {
    return home(axis, Strategy::Min);
}

bool Homing::homeMax(Axis& axis) {
    return home(axis, Strategy::Max);
}

bool Homing::homeMinMax(Axis& axis) {
    return homeMinMax(axis, config_);
}

bool Homing::homeMinMax(Axis& axis, const Config& config) {
    Config cfg = config;
    cfg.strategy = Strategy::MinThenMax;
    return home(axis, cfg);
}

bool Homing::homeMaxMin(Axis& axis) {
    return homeMaxMin(axis, config_);
}

bool Homing::homeMaxMin(Axis& axis, const Config& config) {
    Config cfg = config;
    cfg.strategy = Strategy::MaxThenMin;
    return home(axis, cfg);
}

} // namespace Generic
} // namespace Stepper
