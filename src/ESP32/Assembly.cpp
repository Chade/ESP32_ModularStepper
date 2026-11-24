#include "Assembly.hpp"

#include "Driver_MCPWM.hpp"
#include "Endstop.hpp"
#include "Generator.hpp"
#include "Pin.hpp"
#include "../Generic/Log.hpp"

namespace Stepper {
namespace ESP32 {

namespace {

std::unique_ptr<ESP32::Pin> createPin(const Generic::PinConfig& cfg) {
    return std::make_unique<ESP32::Pin>(cfg.gpio, cfg.inverse, cfg.ioMode, cfg.pullMode);
}

bool isInputCompatible(Generic::Pin::IoMode mode) {
    return mode == Generic::Pin::IoMode::Input || mode == Generic::Pin::IoMode::Input_Output;
}

} // namespace

// =============================================================================
// MotorAssembly
// =============================================================================

std::unique_ptr<MotorAssembly> MotorAssembly::create(const Generic::MotorConfig& config) {
    if (config.driver.microsteps == 0) {
        LOG_ERROR("Invalid MotorConfig: microsteps must be > 0");
        return nullptr;
    }
    if (config.stepsPerRevolution == 0) {
        LOG_ERROR("Invalid MotorConfig: stepsPerRevolution must be > 0");
        return nullptr;
    }
    if (config.gearRatio <= 0.0f) {
        LOG_ERROR("Invalid MotorConfig: gearRatio must be > 0");
        return nullptr;
    }

    std::unique_ptr<MotorAssembly> assembly(new MotorAssembly());

    // 1. Pins
    assembly->pinEnable_    = createPin(config.driver.enablePin);
    assembly->pinStep_      = createPin(config.driver.stepPin);
    assembly->pinDirection_ = createPin(config.driver.directionPin);

    // 2. Driver
    switch (config.driver.type) {
        case Generic::DriverConfig::Type::MCPWM:
            assembly->driver_ = std::make_unique<ESP32::MCPWM::Driver>(
                *assembly->pinEnable_,
                *assembly->pinStep_,
                *assembly->pinDirection_,
                config.driver.microsteps);
            break;
        default:
            LOG_ERROR("Unsupported DriverConfig::Type: %u", static_cast<unsigned>(config.driver.type));
            return nullptr;
    }

    assembly->driver_->setTiming(
        config.driver.minPulseWidthHigh_us,
        config.driver.minPulseWidthLow_us,
        config.driver.directionDelay_us,
        config.driver.enableDelay_us);

    // 3. Generator (registers callback and initializes driver)
    assembly->generator_ = std::make_unique<ESP32::Generator>(*assembly->driver_);

    // 4. Motor
    assembly->motor_ = std::make_unique<Generic::Motor>(
        *assembly->generator_,
        config.stepsPerRevolution,
        config.gearRatio);

    return assembly;
}

MotorAssembly::~MotorAssembly() {
    if (motor_) {
        motor_->stop();
    }
}

// =============================================================================
// AxisAssembly
// =============================================================================

std::unique_ptr<AxisAssembly> AxisAssembly::create(const Generic::AxisConfig& config) {
    auto motorAssembly = MotorAssembly::create(config.motor);
    if (!motorAssembly) {
        return nullptr;
    }
    return create(std::move(motorAssembly), config);
}

std::unique_ptr<AxisAssembly> AxisAssembly::create(std::unique_ptr<MotorAssembly> motorAssembly, const Generic::AxisConfig& config) {
    if (!motorAssembly) {
        LOG_ERROR("Invalid AxisAssembly: motorAssembly is null");
        return nullptr;
    }
    if (config.unitsPerRevolution <= 0.0f) {
        LOG_ERROR("Invalid AxisConfig: unitsPerRevolution must be > 0");
        return nullptr;
    }
    if (config.hasSoftLimits && !(config.minLimit < config.maxLimit)) {
        LOG_ERROR("Invalid AxisConfig: minLimit must be < maxLimit");
        return nullptr;
    }
    if (config.endstopMin.enabled && !isInputCompatible(config.endstopMin.pin.ioMode)) {
        LOG_ERROR("Invalid AxisConfig: endstopMin pin must use Input or Input_Output IoMode");
        return nullptr;
    }
    if (config.endstopMax.enabled && !isInputCompatible(config.endstopMax.pin.ioMode)) {
        LOG_ERROR("Invalid AxisConfig: endstopMax pin must use Input or Input_Output IoMode");
        return nullptr;
    }

    std::unique_ptr<AxisAssembly> assembly(new AxisAssembly());
    assembly->axisType_      = config.type;
    assembly->motorAssembly_ = std::move(motorAssembly);

    // 1. Optional Endstops
    if (config.endstopMin.enabled) {
        assembly->pinEndstopMin_ = createPin(config.endstopMin.pin);
        assembly->endstopMin_    = std::make_unique<ESP32::Endstop>(*assembly->pinEndstopMin_);
        assembly->endstopMin_->setDebounceUs(config.endstopMin.debounceUs);
    }
    if (config.endstopMax.enabled) {
        assembly->pinEndstopMax_ = createPin(config.endstopMax.pin);
        assembly->endstopMax_    = std::make_unique<ESP32::Endstop>(*assembly->pinEndstopMax_);
        assembly->endstopMax_->setDebounceUs(config.endstopMax.debounceUs);
    }

    // 2. Axis (Linear or Rotary)
    switch (config.type) {
        case Generic::AxisConfig::Type::Linear:
            assembly->axis_ = std::make_unique<Generic::LinearAxis>(
                assembly->motorAssembly_->motor(),
                config.unitsPerRevolution);
            break;
        case Generic::AxisConfig::Type::Rotary:
            assembly->axis_ = std::make_unique<Generic::RotaryAxis>(
                assembly->motorAssembly_->motor(),
                config.unitsPerRevolution,
                config.continuous);
            break;
        default:
            LOG_ERROR("Unsupported AxisConfig::Type: %u", static_cast<unsigned>(config.type));
            return nullptr;
    }

    // 3. Attach Endstops & Soft Limits
    if (assembly->endstopMin_) {
        assembly->axis_->setEndstopMin(assembly->endstopMin_.get(), config.endstopMin.limitProtection);
    }
    if (assembly->endstopMax_) {
        assembly->axis_->setEndstopMax(assembly->endstopMax_.get(), config.endstopMax.limitProtection);
    }
    if (config.hasSoftLimits) {
        assembly->axis_->setLimits(config.minLimit, config.maxLimit);
    }

    return assembly;
}

AxisAssembly::~AxisAssembly() {
    if (axis_) {
        axis_->stop();
    }
    // Unregister endstop callbacks before destroying axis_ so ISRs/tasks cannot invoke callbacks on a destroyed Axis
    if (endstopMin_) {
        endstopMin_->unregisterCallback();
    }
    if (endstopMax_) {
        endstopMax_->unregisterCallback();
    }
}

} // namespace ESP32
} // namespace Stepper
