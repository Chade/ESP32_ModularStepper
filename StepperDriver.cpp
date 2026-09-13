#include "StepperDriver.hpp"
#include "StepperLog.hpp"

namespace Stepper {
namespace Generic {
    Driver::Driver(Pin& enablePin, Pin& stepPin, Pin& directionPin, uint8_t microsteps)
            : pinEnable_(enablePin), pinStep_(stepPin), pinDirection_(directionPin), microsteps_(microsteps) {

    }

    void Driver::enable() {
        pinEnable_.enable();
    }

    void Driver::disable() {
        pinEnable_.disable();
    }

    bool Driver::isEnabled() const {
        return pinEnable_.isEnabled();
    }

    void Driver::reset(bool resetSteps) {
        if (isRunning()) {
            stop();
        }

        pulsePeriod_us_ = 0.0f;

        if (resetSteps) {
            numStepsDone_ = 0;
        }
    }

    void Driver::setDirection(Direction direction) {
        if (direction == Direction::Counterclockwise) {
            pinDirection_.enable();
        }
        else if (direction == Direction::Clockwise) {
            pinDirection_.disable();
        }
        else {
            disable();
        }
    }

    Direction Driver::getDirection() const
    {
        if (!isEnabled()) {
            return Direction::Neutral;
        }

        if (pinDirection_.isEnabled()) {
            return Direction::Counterclockwise;
        }
        else {
            return Direction::Clockwise;
        }
    }

    Direction Driver::changeDirection()
    {
        if (!isEnabled()) {
            return Direction::Neutral;
        }

        if (getDirection() == Direction::Clockwise) {
            setDirection(Direction::Counterclockwise);
            return Direction::Counterclockwise;
        }
        else if (getDirection() == Direction::Counterclockwise) {
            setDirection(Direction::Clockwise);
            return Direction::Clockwise;
        }
        return Direction::Neutral;
    }

    void Driver::setTiming(float minPulseWidthHigh_us, float minPulseWidthLow_us, float directionDelay_us, float enableDelay_us)
    {
        minPulseWidthHigh_us_ = minPulseWidthHigh_us;
        minPulseWidthLow_us_ = minPulseWidthLow_us;
        directionDelay_us_ = directionDelay_us;
        enableDelay_us_ = enableDelay_us;
    }

    float Driver::getMinPulsePeriodUs() const
    {
        return minPulsePeriod_us_;
    }

    float Driver::getMaxPulsePeriodUs() const
    {
        return maxPulsePeriod_us_;
    }

    bool Driver::setPulsePeriodUs(float pulsePeriod_us) {
        return checkPulsePeriod(pulsePeriod_us, pulsePeriod_us_);
    }

    float Driver::getPulsePeriodUs() const {
        return pulsePeriod_us_;
    }
    
    bool Driver::checkPulsePeriod(float pulsePeriod_us) {
        float pulsePeriodNew_us = 0.0;
        return checkPulsePeriod(pulsePeriod_us, pulsePeriodNew_us);
    }

    bool Driver::checkPulsePeriod(float pulsePeriod_us, float& pulePeriodNew_us) {
        float minPulseWidthHigh_us = std::max(minPulseWidthHigh_us_, minPulsePeriod_us_);
        float minPulseWidthLow_us = std::max(minPulseWidthLow_us_, minPulsePeriod_us_);
        
        if (pulsePeriod_us < (minPulseWidthHigh_us + minPulseWidthLow_us)) {
            LOG_WARN("Pulse period %.2f us is shorter than allowed period: %.2f us", pulsePeriod_us, (minPulseWidthHigh_us + minPulseWidthLow_us));
            pulePeriodNew_us = (minPulseWidthHigh_us + minPulseWidthLow_us);
            return false;
        }
        else if (pulsePeriod_us > maxPulsePeriod_us_) {
            LOG_WARN("Pulse period %.2f us is longer than allowed pulse period %.2f us", pulsePeriod_us, maxPulsePeriod_us_);
            pulePeriodNew_us = maxPulsePeriod_us_;
            return false;
        }
        pulePeriodNew_us = pulsePeriod_us;
        return true;
    }

    uint8_t Driver::getMicrosteps() const {
        return microsteps_;
    }

    uint64_t Driver::getSteps() const {
        return numStepsDone_;
    }

    void Driver::resetSteps(uint64_t count) {
        numStepsDone_ = count;
    }

    void Driver::registerCallbackOnStepDone(DriverCallback callback, void* user_ctx) {
        callbackOnStepDoneUserCtx_ = user_ctx;
        callbackOnStepDone_ = std::move(callback);
    }

} // namespace Generic
} // namespace Stepper