#ifndef STEPPER_AXIS_HPP
#define STEPPER_AXIS_HPP

#include "StepperMotor.hpp"
#include <functional>

namespace Stepper {
    class AxisBase {
    public:
        using HomeSensorCallback = std::function<bool(void*)>;

        AxisBase() = delete;
        explicit AxisBase(Motor& motor);
        virtual ~AxisBase() = default;

        virtual bool moveTo(float targetAxisUnit,
                            float velocityAxisUnitPerSec,
                            float accelerationAxisUnitPerSec2,
                            float decelerationAxisUnitPerSec2);

        virtual bool moveBy(float deltaAxisUnit,
                            float velocityAxisUnitPerSec,
                            float accelerationAxisUnitPerSec2,
                            float decelerationAxisUnitPerSec2);

        virtual bool setVelocity(float velocityAxisUnitPerSec,
                                 float accelerationAxisUnitPerSec2,
                                 float decelerationAxisUnitPerSec2);

        virtual void stop(float decelerationAxisUnitPerSec2);
        virtual void emergencyStop();

        virtual bool home(float homingVelocityAxisUnitPerSec,
                          float homingAccelerationAxisUnitPerSec2,
                          float homingDecelerationAxisUnitPerSec2,
                          Direction direction,
                          float zeroOffsetAxisUnit = 0.0f);

        virtual void setZero(float axisUnit = 0.0f);
        virtual bool isHomed() const;

        virtual float getPosition() const;
        virtual float getVelocity() const;
        virtual State getState() const;

        virtual void update();

        void registerHomeSensorCallback(HomeSensorCallback callback, void* userCtx = nullptr);

        Motor& getMotor();
        const Motor& getMotor() const;

    protected:
        virtual float axisUnitToRevolution(float axisUnit) const = 0;
        virtual float revolutionToAxisUnit(float revolution) const = 0;

        float stepsToAxisUnit(uint64_t steps) const;

        enum class CommandMode : uint8_t {
            Idle = 0,
            Position,
            Velocity,
            Homing
        };

        void syncPositionFromMotion() const;

        Motor& motor_;

        mutable float positionAxisUnit_ = 0.0f;
        mutable int64_t commandStartPositionUs_ = 0;

        Direction activeDirection_ = Direction::Neutral;
        CommandMode commandMode_ = CommandMode::Idle;

        float moveStartPositionAxisUnit_ = 0.0f;
        float moveTargetPositionAxisUnit_ = 0.0f;
        float moveOffsetAxisUnit_ = 0.0f;

        float velocityAxisUnitPerSec_ = 0.0f;
        float homingZeroOffsetAxisUnit_ = 0.0f;

        bool homed_ = false;

        HomeSensorCallback homeSensorCallback_ = nullptr;
        void* homeSensorUserCtx_ = nullptr;
    };

    class RotaryAxis : public AxisBase {
    public:
        RotaryAxis() = delete;
        explicit RotaryAxis(Motor& motor);
        ~RotaryAxis() override = default;

    protected:
        float axisUnitToRevolution(float angle_deg) const override;
        float revolutionToAxisUnit(float revolution) const override;
    };

    class LinearAxis : public AxisBase {
    public:
        struct LinearParams {
            float meterPerRevolution = 1.0f;
        };

        LinearAxis() = delete;
        LinearAxis(Motor& motor, const LinearParams& params);
        ~LinearAxis() override = default;

        void setLinearParams(const LinearParams& params);
        LinearParams getLinearParams() const;

    protected:
        float axisUnitToRevolution(float meters) const override;
        float revolutionToAxisUnit(float revolution) const override;

    private:
        LinearParams params_;
    };

    // Backward-compatible alias: default Axis uses rotary units (degrees).
    class Axis : public RotaryAxis {
    public:
        Axis() = delete;
        explicit Axis(Motor& motor) : RotaryAxis(motor) {}
        ~Axis() override = default;
    };
}

#endif // STEPPER_AXIS_HPP