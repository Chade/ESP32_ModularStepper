#ifndef STEPPER_ASSEMBLY_ESP32_HPP
#define STEPPER_ASSEMBLY_ESP32_HPP

#include <memory>

#include "../Generic/Axis.hpp"
#include "../Generic/Axis_Linear.hpp"
#include "../Generic/Axis_Rotary.hpp"
#include "../Generic/Config.hpp"
#include "../Generic/Driver.hpp"
#include "../Generic/Endstop.hpp"
#include "../Generic/Generator.hpp"
#include "../Generic/Motor.hpp"
#include "../Generic/Pin.hpp"
#include "../Generic/StopController.hpp"

namespace Stepper {
namespace ESP32 {

// Smallest modular runtime unit: owns Pins -> Driver -> Generator -> Motor
// Members are declared in strict dependency order so C++ destroys them in exact reverse order.
class MotorAssembly {
public:
    static std::unique_ptr<MotorAssembly> create(const Generic::MotorConfig& config,
                                                 Generic::StopController* stopController = nullptr,
                                                 Generic::Pin* sharedEnablePin = nullptr);

    ~MotorAssembly();

    MotorAssembly(const MotorAssembly&) = delete;
    MotorAssembly& operator=(const MotorAssembly&) = delete;
    MotorAssembly(MotorAssembly&&) = delete;
    MotorAssembly& operator=(MotorAssembly&&) = delete;

    Generic::Motor& motor() { return *motor_; }
    const Generic::Motor& motor() const { return *motor_; }

    Generic::Generator& generator() { return *generator_; }
    const Generic::Generator& generator() const { return *generator_; }

    Generic::Driver& driver() { return *driver_; }
    const Generic::Driver& driver() const { return *driver_; }

    Generic::Pin* pinEnable() { return enablePinPtr_; }
    const Generic::Pin* pinEnable() const { return enablePinPtr_; }

    Generic::Pin& pinStep() { return *pinStep_; }
    const Generic::Pin& pinStep() const { return *pinStep_; }

    Generic::Pin& pinDirection() { return *pinDirection_; }
    const Generic::Pin& pinDirection() const { return *pinDirection_; }

private:
    MotorAssembly() = default;

    // Declaration order == construction order; reverse == destruction order
    Generic::StopController* stopController_ {nullptr};
    int16_t registeredId_ {-1};
    std::unique_ptr<Generic::Pin> pinEnable_;
    Generic::Pin* enablePinPtr_ {nullptr};
    std::unique_ptr<Generic::Pin> pinStep_;
    std::unique_ptr<Generic::Pin> pinDirection_;
    std::unique_ptr<Generic::Driver> driver_;
    std::unique_ptr<Generic::Generator> generator_;
    std::unique_ptr<Generic::Motor> motor_;
};

// Base class for axis assemblies: composes a MotorAssembly + optional Endstops + Generic::Axis
class AxisAssembly {
public:
    virtual ~AxisAssembly();

    AxisAssembly(const AxisAssembly&) = delete;
    AxisAssembly& operator=(const AxisAssembly&) = delete;
    AxisAssembly(AxisAssembly&&) = delete;
    AxisAssembly& operator=(AxisAssembly&&) = delete;

    Generic::Axis& axis() { return *axis_; }
    const Generic::Axis& axis() const { return *axis_; }

    MotorAssembly& motorAssembly() { return *motorAssembly_; }
    const MotorAssembly& motorAssembly() const { return *motorAssembly_; }

    // Forwarding accessors to the underlying MotorAssembly
    Generic::Motor& motor() { return motorAssembly_->motor(); }
    const Generic::Motor& motor() const { return motorAssembly_->motor(); }

    Generic::Generator& generator() { return motorAssembly_->generator(); }
    const Generic::Generator& generator() const { return motorAssembly_->generator(); }

    Generic::Driver& driver() { return motorAssembly_->driver(); }
    const Generic::Driver& driver() const { return motorAssembly_->driver(); }

    Generic::Endstop* endstopMin() { return endstopMin_.get(); }
    const Generic::Endstop* endstopMin() const { return endstopMin_.get(); }

    Generic::Endstop* endstopMax() { return endstopMax_.get(); }
    const Generic::Endstop* endstopMax() const { return endstopMax_.get(); }

protected:
    AxisAssembly() = default;

    bool initMotorAndEndstops(std::unique_ptr<MotorAssembly> motorAssembly, const Generic::AxisConfig& config);
    void attachEndstopsAndLimits(const Generic::AxisConfig& config);
    bool registerWithStopController(const Generic::AxisConfig& config, Generic::StopController* stopController);

    // Declaration order == construction order; reverse == destruction order
    // motorAssembly_ and endstops outlive axis_, and endstop pins outlive endstops.
    Generic::StopController* stopController_ {nullptr};
    int16_t registeredId_ {-1};
    std::unique_ptr<MotorAssembly> motorAssembly_;
    std::unique_ptr<Generic::Pin> pinEndstopMin_;
    std::unique_ptr<Generic::Pin> pinEndstopMax_;
    std::unique_ptr<Generic::Endstop> endstopMin_;
    std::unique_ptr<Generic::Endstop> endstopMax_;
    std::unique_ptr<Generic::Axis> axis_;
};

// Type-safe assembly for LinearAxis
class LinearAxisAssembly : public AxisAssembly {
public:
    static std::unique_ptr<LinearAxisAssembly> create(const Generic::LinearAxisConfig& config,
                                                      Generic::StopController* stopController = nullptr,
                                                      Generic::Pin* sharedEnablePin = nullptr);
    static std::unique_ptr<LinearAxisAssembly> create(std::unique_ptr<MotorAssembly> motorAssembly,
                                                      const Generic::LinearAxisConfig& config,
                                                      Generic::StopController* stopController = nullptr);

    Generic::LinearAxis& axis() { return static_cast<Generic::LinearAxis&>(*axis_); }
    const Generic::LinearAxis& axis() const { return static_cast<const Generic::LinearAxis&>(*axis_); }

private:
    LinearAxisAssembly() = default;
};

// Type-safe assembly for RotaryAxis
class RotaryAxisAssembly : public AxisAssembly {
public:
    static std::unique_ptr<RotaryAxisAssembly> create(const Generic::RotaryAxisConfig& config,
                                                      Generic::StopController* stopController = nullptr,
                                                      Generic::Pin* sharedEnablePin = nullptr);
    static std::unique_ptr<RotaryAxisAssembly> create(std::unique_ptr<MotorAssembly> motorAssembly,
                                                      const Generic::RotaryAxisConfig& config,
                                                      Generic::StopController* stopController = nullptr);

    Generic::RotaryAxis& axis() { return static_cast<Generic::RotaryAxis&>(*axis_); }
    const Generic::RotaryAxis& axis() const { return static_cast<const Generic::RotaryAxis&>(*axis_); }

private:
    RotaryAxisAssembly() = default;
};

} // namespace ESP32
} // namespace Stepper

#endif // STEPPER_ASSEMBLY_ESP32_HPP
