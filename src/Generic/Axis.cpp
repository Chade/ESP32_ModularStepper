#include "Axis.hpp"
#include "Log.hpp"

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

bool Axis::pause() {
    return motor_.pause();
}

bool Axis::resume() {
    return motor_.resume();
}

void Axis::fastStop() {
    motor_.fastStop();
}

void Axis::emergencyStop(bool disableDriver) {
    motor_.emergencyStop(disableDriver);
    isHoming_ = false;
    isHomed_ = false;
}

bool Axis::clearEmergencyStop(bool enableDriver) {
    return motor_.clearEmergencyStop(enableDriver);
}

bool Axis::isDirectionAllowed(Direction direction) const {
    if (isHoming_) {
        return true;
    }

    if (limitProtectionMinEnabled_ && isEndstopTriggered(EndstopPosition::Min) &&
        compare_enums(EndstopPosition::Min, direction)) {
        return false;
    }

    if (limitProtectionMaxEnabled_ && isEndstopTriggered(EndstopPosition::Max) &&
        compare_enums(EndstopPosition::Max, direction)) {
        return false;
    }

    return true;
}

bool Axis::enableLimitProtection(EndstopPosition pos, bool enable) {
    if (pos == EndstopPosition::Min) {
        return enableLimitProtectionMin(enable);
    }
    else if (pos == EndstopPosition::Max) {
        return enableLimitProtectionMax(enable);
    }

    return false;
}

bool Axis::enableLimitProtectionMin(bool enable) {
    if (endstopMin_ == nullptr) {
        LOG_ERROR("No endstop at position MIN");
        limitProtectionMinEnabled_ = false;
        return false;
    }

    if (enable && (!limitProtectionMinEnabled_ || !endstopMin_->hasCallback())) {
        endstopMin_->registerCallback(Generic::Endstop::Edge::Both, [this](Generic::Endstop::Edge) {
            if (limitProtectionMinEnabled_ && endstopMin_->isTriggered()) {
                Direction direction = motor_.generator().getState().currentDirection;
                if (compare_enums(EndstopPosition::Min, direction)) {
                    stop();
                    LOG_WARN("Motion halted: Min limit switch triggered!");
                }
            }
        });
    }
    limitProtectionMinEnabled_ = enable;
    return true;
}

bool Axis::enableLimitProtectionMax(bool enable) {
    if (endstopMax_ == nullptr) {
        LOG_ERROR("No endstop at position MAX");
        limitProtectionMaxEnabled_ = false;
        return false;
    }

    if (enable && (!limitProtectionMaxEnabled_ || !endstopMax_->hasCallback())) {
        endstopMax_->registerCallback(Generic::Endstop::Edge::Both, [this](Generic::Endstop::Edge) {
            if (limitProtectionMaxEnabled_ && endstopMax_->isTriggered()) {
                Direction direction = motor_.generator().getState().currentDirection;
                if (compare_enums(EndstopPosition::Max, direction)) {
                    stop();
                    LOG_WARN("Motion halted: Max limit switch triggered!");
                }
            }
        });
    }
    limitProtectionMaxEnabled_ = enable;
    return true;
}

} // namespace Generic
} // namespace Stepper
