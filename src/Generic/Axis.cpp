#include "Axis.hpp"

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
}

bool Axis::isDirectionAllowed(Direction direction) const {
    if (isHoming_) {
        return true;
    }

    if (limitProtectionMinEnabled_ && isEndstopTriggered(EndstopPosition::Min) &&
        compare_enums(EndstopPosition::Min, direction)) {
        LOG_WARN("Move blocked: Min limit switch is triggered");
        return false;
    }

    if (limitProtectionMaxEnabled_ && isEndstopTriggered(EndstopPosition::Max) &&
        compare_enums(EndstopPosition::Max, direction)) {
        LOG_WARN("Move blocked: Max limit switch is triggered");
        return false;
    }

    return true;
}

void Axis::enableLimitProtectionMin(bool enable) {
    limitProtectionMinEnabled_ = enable;
    if (enable && endstopMin_) {
        endstopMin_->registerCallback(Generic::Endstop::Edge::Both, [this](Generic::Endstop::Edge) {
            Direction curDir = motor_.generator().getState().currentDirection;
            if (limitProtectionMinEnabled_ && !isHoming_ && endstopMin_->isTriggered() &&
                !compare_enums(EndstopPosition::Max, curDir)) {
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
            Direction curDir = motor_.generator().getState().currentDirection;
            if (limitProtectionMaxEnabled_ && !isHoming_ && endstopMax_->isTriggered() &&
                !compare_enums(EndstopPosition::Min, curDir)) {
                LOG_WARN("Max limit switch triggered! Halting motion.");
                stop();
            }
        });
    }
}

} // namespace Generic
} // namespace Stepper
