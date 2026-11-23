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
    // Enable endstop
    if (!axis.enableLimitProtection(endstopPos, true)) {
        return false;
    }

    // Step 0: Backoff from endstop, if endstop is already triggered
    state_ = State::BackingOff;
    if (!backoffFromEndstop(axis, endstopPos, config.slowVelocity, config.acceleration, config.deceleration, config.backoffDistance)) {
        return false;
    }

    // Step 1: Fast Approach (bounded by maxTravelDistance if > 0)
    state_ = State::FastApproach;
    if (!approachEndstop(axis, endstopPos, config.fastVelocity, config.acceleration, config.deceleration, config.maxTravelDistance)) {
        return false;
    }

    // Step 2: Retract by retractDistance
    state_ = State::Retracting;
    if (!backoffFromEndstop(axis, endstopPos, config.slowVelocity, config.acceleration, config.deceleration, config.retractDistance)) {
        return false;
    }

    // Step 3: Slow Approach (bounded by retractDistance * 1.5f)
    state_ = State::SlowApproach;
    if (!approachEndstop(axis, endstopPos, config.slowVelocity, config.acceleration, config.deceleration, config.retractDistance * 1.5f)) {
        return false;
    }

    return true;
}

bool Homing::approachEndstop(Axis& axis, Axis::EndstopPosition endstopPos, float velocity, float acceleration, float deceleration, float distance) {
    Generic::Endstop* targetEndstop = axis.getEndstop(endstopPos);

    if (targetEndstop == nullptr) {
        LOG_ERROR("No endstop at position %s!", (endstopPos == Axis::EndstopPosition::Min ? "MIN" : "MAX"));
        return false;
    }
    
    if (targetEndstop->isTriggered()) {
        return true;
    }

    Direction direction = searchDirectionFor(endstopPos);
    float signedVelocity = fabs(velocity) * sign(direction);
    if (distance > 0.0f) {
        axis.run(distance, signedVelocity, acceleration, deceleration);
    } else {
        axis.run(signedVelocity, acceleration, deceleration);
    }
    axis.waitForState(Stepper::State::Stopped);

    if (!targetEndstop->isTriggered()) {
        LOG_ERROR("Endstop approach exceeded max travel distance (%.2f)!", distance);
        return false;
    }
    return true;
}

bool Homing::backoffFromEndstop(Axis& axis, Axis::EndstopPosition endstopPos, float velocity, float acceleration, float dceleration, float distance) {
    if (distance == 0.0f) {
        LOG_ERROR("Backoff distance must not be zero!");
        return false;
    }

    Generic::Endstop* targetEndstop = axis.getEndstop(endstopPos);

    if (targetEndstop == nullptr) {
        LOG_ERROR("No endstop at position %s!", (endstopPos == Axis::EndstopPosition::Min ? "MIN" : "MAX"));
        return false;
    }

    if (!targetEndstop->isTriggered()) {
        return true;
    }

    Direction direction = reverseDirection(searchDirectionFor(endstopPos));
    float signedVelocity = fabs(velocity) * sign(direction);
    axis.run(distance, signedVelocity, acceleration, dceleration);
    axis.waitForState(Stepper::State::Stopped);

    if (targetEndstop->isTriggered()) {
        LOG_ERROR("Endstop still triggered after backing off (%.2f)!", distance);
        return false;
    }

    return true;
}


bool Homing::home(Axis& axis, const Config& config) {
    if (isDualEndstopStrategy(config.strategy)) {
        if (!axis.hasEndstopMin() || !axis.hasEndstopMax()) {
            LOG_ERROR("Cannot perform dual-endstop homing: Both Min and Max endstops must be configured!");
            state_ = State::Failed;
            return false;
        }

        const bool minFirst = (config.strategy == Strategy::MinThenMax);
        const Axis::EndstopPosition firstEndstop  = minFirst ? Axis::EndstopPosition::Min : Axis::EndstopPosition::Max;
        const Axis::EndstopPosition secondEndstop = minFirst ? Axis::EndstopPosition::Max : Axis::EndstopPosition::Min;

        axis.setHomed(false);
        axis.setHoming(true);

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

        float measuredPosition = axis.getPosition();
        float minPos = minFirst ? config.homePosition : measuredPosition;
        float maxPos = minFirst ? measuredPosition : config.homePosition;
        if (maxPos <= minPos) {
            LOG_ERROR("Invalid calibrated limits [%.2f, %.2f]", minPos, maxPos);
            state_ = State::Failed;
            axis.setHoming(false);
            return false;
        }
        axis.setLimits(minPos, maxPos);

        // 3. Final back-off off the second switch
        if (!backoffFromEndstop(axis, secondEndstop, config.slowVelocity, config.acceleration, config.deceleration, config.backoffDistance)) {
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
        LOG_WARN("Cannot perform homing: Endstop %s must be configured!",
                (targetEndstop == Axis::EndstopPosition::Min) ? "Min" : "Max");
        state_ = State::Failed;
        return false;
    }

    axis.setHomed(false);
    axis.setHoming(true);

    LOG_INFO("Single-endstop homing: Homing to %s endstop...", 
            (targetEndstop == Axis::EndstopPosition::Min) ? "Min" : "Max");
    if (!homeToEndstop(axis, targetEndstop, config)) {
        state_ = State::Failed;
        axis.setHoming(false);
        return false;
    }
    axis.setPosition(config.homePosition);

    if (!backoffFromEndstop(axis, targetEndstop, config.slowVelocity, config.acceleration, config.deceleration, config.backoffDistance)) {
        state_ = State::Failed;
        axis.setHoming(false);
        return false;
    }

    finalizeAxisState(axis);
    LOG_INFO("Single-endstop homing successful. Position: %.2f (home ref: %.2f)",
            axis.getPosition(), config.homePosition);
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
