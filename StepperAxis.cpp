#include "StepperAxis.hpp"

#include <esp_timer.h>
#include <cmath>

namespace Stepper {

    AxisBase::AxisBase(Motor& motor)
        : motor_(motor) {

    }

    bool AxisBase::moveTo(float targetAxisUnit,
                          float velocityAxisUnitPerSec,
                          float accelerationAxisUnitPerSec2,
                          float decelerationAxisUnitPerSec2) {
        if (!homed_) {
            return false;
        }

        syncPositionFromMotion();

        float deltaAxisUnit = targetAxisUnit - positionAxisUnit_;
        return moveBy(deltaAxisUnit, velocityAxisUnitPerSec, accelerationAxisUnitPerSec2, decelerationAxisUnitPerSec2);
    }

    bool AxisBase::moveBy(float deltaAxisUnit,
                          float velocityAxisUnitPerSec,
                          float accelerationAxisUnitPerSec2,
                          float decelerationAxisUnitPerSec2) {
        if (velocityAxisUnitPerSec <= 0.0f || accelerationAxisUnitPerSec2 <= 0.0f || decelerationAxisUnitPerSec2 <= 0.0f) {
            return false;
        }

        syncPositionFromMotion();

        if (deltaAxisUnit == 0.0f) {
            commandMode_ = CommandMode::Idle;
            return true;
        }

        Direction direction = (deltaAxisUnit > 0.0f) ? Direction::Counterclockwise : Direction::Clockwise;

        float deltaRev = axisUnitToRevolution(std::fabs(deltaAxisUnit));
        float velRev = axisUnitToRevolution(velocityAxisUnitPerSec);
        float accRev = axisUnitToRevolution(accelerationAxisUnitPerSec2);
        float decRev = axisUnitToRevolution(decelerationAxisUnitPerSec2);

        bool started = motor_.run(deltaRev * 360.0f, velRev, accRev, decRev, direction);
        if (!started) {
            return false;
        }

        moveStartPositionAxisUnit_ = positionAxisUnit_;
        moveOffsetAxisUnit_ = std::fabs(deltaAxisUnit);
        moveTargetPositionAxisUnit_ = positionAxisUnit_ + deltaAxisUnit;

        activeDirection_ = direction;
        commandMode_ = CommandMode::Position;
        commandStartPositionUs_ = esp_timer_get_time();
        velocityAxisUnitPerSec_ = velocityAxisUnitPerSec;

        return true;
    }

    bool AxisBase::setVelocity(float velocityAxisUnitPerSec,
                               float accelerationAxisUnitPerSec2,
                               float decelerationAxisUnitPerSec2) {
        if (velocityAxisUnitPerSec < 0.0f || accelerationAxisUnitPerSec2 <= 0.0f || decelerationAxisUnitPerSec2 <= 0.0f) {
            return false;
        }

        syncPositionFromMotion();

        Direction direction = activeDirection_;
        if (velocityAxisUnitPerSec == 0.0f) {
            commandMode_ = CommandMode::Idle;
            return motor_.run(0.0f, axisUnitToRevolution(accelerationAxisUnitPerSec2), axisUnitToRevolution(decelerationAxisUnitPerSec2), direction);
        }

        if (direction == Direction::Neutral) {
            direction = Direction::Counterclockwise;
        }

        bool started = motor_.run(axisUnitToRevolution(velocityAxisUnitPerSec),
                                  axisUnitToRevolution(accelerationAxisUnitPerSec2),
                                  axisUnitToRevolution(decelerationAxisUnitPerSec2),
                                  direction);
        if (!started) {
            return false;
        }

        activeDirection_ = direction;
        commandMode_ = CommandMode::Velocity;
        commandStartPositionUs_ = esp_timer_get_time();
        velocityAxisUnitPerSec_ = velocityAxisUnitPerSec;

        return true;
    }

    void AxisBase::stop(float decelerationAxisUnitPerSec2) {
        if (decelerationAxisUnitPerSec2 <= 0.0f) {
            decelerationAxisUnitPerSec2 = 1.0f;
        }

        syncPositionFromMotion();

        float currentVelocity = std::fabs(getVelocity());
        motor_.run(0.0f,
                   axisUnitToRevolution(currentVelocity),
                   axisUnitToRevolution(decelerationAxisUnitPerSec2),
                   activeDirection_);
        commandMode_ = CommandMode::Idle;
    }

    void AxisBase::emergencyStop() {
        syncPositionFromMotion();

        motor_.getGenerator().getDriver().stop();
        commandMode_ = CommandMode::Idle;
    }

    bool AxisBase::home(float homingVelocityAxisUnitPerSec,
                        float homingAccelerationAxisUnitPerSec2,
                        float homingDecelerationAxisUnitPerSec2,
                        Direction direction,
                        float zeroOffsetAxisUnit) {
        if (homeSensorCallback_ == nullptr || direction == Direction::Neutral) {
            return false;
        }

        homingZeroOffsetAxisUnit_ = zeroOffsetAxisUnit;

        bool started = motor_.run(axisUnitToRevolution(homingVelocityAxisUnitPerSec),
                                  axisUnitToRevolution(homingAccelerationAxisUnitPerSec2),
                                  axisUnitToRevolution(homingDecelerationAxisUnitPerSec2),
                                  direction);

        if (started) {
            activeDirection_ = direction;
            commandMode_ = CommandMode::Homing;
            commandStartPositionUs_ = esp_timer_get_time();
            velocityAxisUnitPerSec_ = homingVelocityAxisUnitPerSec;
        }

        return started;
    }

    void AxisBase::setZero(float axisUnit) {
        syncPositionFromMotion();

        positionAxisUnit_ = axisUnit;
        moveStartPositionAxisUnit_ = axisUnit;
        moveTargetPositionAxisUnit_ = axisUnit;
        moveOffsetAxisUnit_ = 0.0f;
        commandStartPositionUs_ = esp_timer_get_time();

        homed_ = true;
    }

    bool AxisBase::isHomed() const {
        return homed_;
    }

    float AxisBase::getPosition() const {
        syncPositionFromMotion();
        return positionAxisUnit_;
    }

    float AxisBase::getVelocity() const {
        float velocity = revolutionToAxisUnit(motor_.getVelocity());
        if (activeDirection_ == Direction::Clockwise) {
            velocity *= -1.0f;
        }
        return velocity;
    }

    State AxisBase::getState() const {
        return motor_.getState();
    }

    void AxisBase::update() {
        syncPositionFromMotion();

        if (commandMode_ == CommandMode::Homing && homeSensorCallback_ != nullptr && homeSensorCallback_(homeSensorUserCtx_)) {
            emergencyStop();
            setZero(homingZeroOffsetAxisUnit_);
        }
    }

    void AxisBase::registerHomeSensorCallback(HomeSensorCallback callback, void* userCtx) {
        homeSensorCallback_ = callback;
        homeSensorUserCtx_ = userCtx;
    }

    Motor& AxisBase::getMotor() {
        return motor_;
    }

    const Motor& AxisBase::getMotor() const {
        return motor_;
    }

    float AxisBase::stepsToAxisUnit(uint64_t steps) const {
        float rev = motor_.stepsToRevolution(static_cast<float>(steps));
        return revolutionToAxisUnit(rev);
    }

    void AxisBase::syncPositionFromMotion() const {
        State state = motor_.getState();

        if (commandMode_ == CommandMode::Position || commandMode_ == CommandMode::Homing) {
            float signedDistance = stepsToAxisUnit(motor_.getGenerator().getStepsDone());
            if (activeDirection_ == Direction::Clockwise) {
                signedDistance *= -1.0f;
            }
            positionAxisUnit_ = moveStartPositionAxisUnit_ + signedDistance;

            if (state == State::Stopped) {
                if (commandMode_ == CommandMode::Position) {
                    positionAxisUnit_ = moveTargetPositionAxisUnit_;
                }
                commandMode_ = CommandMode::Idle;
                moveOffsetAxisUnit_ = 0.0f;
            }
            return;
        }

        if (commandMode_ == CommandMode::Velocity) {
            int64_t nowUs = esp_timer_get_time();
            int64_t dtUs = nowUs - commandStartPositionUs_;
            if (dtUs > 0) {
                float dt_s = static_cast<float>(dtUs) / 1000000.0f;
                positionAxisUnit_ += getVelocity() * dt_s;
                commandStartPositionUs_ = nowUs;
            }

            if (state == State::Stopped) {
                commandMode_ = CommandMode::Idle;
            }
        }
    }

    RotaryAxis::RotaryAxis(Motor& motor)
        : AxisBase(motor) {

    }

    float RotaryAxis::axisUnitToRevolution(float angle_deg) const {
        return angle_deg / 360.0f;
    }

    float RotaryAxis::revolutionToAxisUnit(float revolution) const {
        return revolution * 360.0f;
    }

    LinearAxis::LinearAxis(Motor& motor, const LinearParams& params)
        : AxisBase(motor)
        , params_(params) {
        if (params_.meterPerRevolution <= 0.0f) {
            params_.meterPerRevolution = 1.0f;
        }
    }

    void LinearAxis::setLinearParams(const LinearParams& params) {
        if (params.meterPerRevolution <= 0.0f) {
            return;
        }

        params_ = params;
    }

    LinearAxis::LinearParams LinearAxis::getLinearParams() const {
        return params_;
    }

    float LinearAxis::axisUnitToRevolution(float meters) const {
        return meters / params_.meterPerRevolution;
    }

    float LinearAxis::revolutionToAxisUnit(float revolution) const {
        return revolution * params_.meterPerRevolution;
    }

} // namespace Stepper
