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

    if (config.fastStopDeceleration > 0.0f) {
        assembly->motor_->setFastStopDeceleration(config.fastStopDeceleration);
    }

    return assembly;
}

MotorAssembly::~MotorAssembly() {
    if (motor_) {
        motor_->stop();
    }
}

// =============================================================================
// AxisAssembly (shared base)
// =============================================================================

bool AxisAssembly::initMotorAndEndstops(std::unique_ptr<MotorAssembly> motorAssembly, const Generic::AxisConfig& config) {
    if (!motorAssembly) {
        LOG_ERROR("Invalid AxisAssembly: motorAssembly is null");
        return false;
    }
    if (config.hasSoftLimits && !(config.minLimit < config.maxLimit)) {
        LOG_ERROR("Invalid AxisConfig: minLimit must be < maxLimit");
        return false;
    }
    if (config.endstopMin.enabled && !isInputCompatible(config.endstopMin.pin.ioMode)) {
        LOG_ERROR("Invalid AxisConfig: endstopMin pin must use Input or Input_Output IoMode");
        return false;
    }
    if (config.endstopMax.enabled && !isInputCompatible(config.endstopMax.pin.ioMode)) {
        LOG_ERROR("Invalid AxisConfig: endstopMax pin must use Input or Input_Output IoMode");
        return false;
    }

    motorAssembly_ = std::move(motorAssembly);

    if (config.endstopMin.enabled) {
        pinEndstopMin_ = createPin(config.endstopMin.pin);
        endstopMin_    = std::make_unique<ESP32::Endstop>(*pinEndstopMin_);
        endstopMin_->setDebounceUs(config.endstopMin.debounceUs);
    }
    if (config.endstopMax.enabled) {
        pinEndstopMax_ = createPin(config.endstopMax.pin);
        endstopMax_    = std::make_unique<ESP32::Endstop>(*pinEndstopMax_);
        endstopMax_->setDebounceUs(config.endstopMax.debounceUs);
    }

    return true;
}

void AxisAssembly::attachEndstopsAndLimits(const Generic::AxisConfig& config) {
    if (endstopMin_) {
        axis_->setEndstopMin(endstopMin_.get(), config.endstopMin.limitProtection);
    }
    if (endstopMax_) {
        axis_->setEndstopMax(endstopMax_.get(), config.endstopMax.limitProtection);
    }
    if (config.hasSoftLimits) {
        axis_->setLimits(config.minLimit, config.maxLimit);
    }
    if (config.fastStopDeceleration > 0.0f) {
        axis_->setFastStopDeceleration(config.fastStopDeceleration);
    }
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

// =============================================================================
// LinearAxisAssembly
// =============================================================================

std::unique_ptr<LinearAxisAssembly> LinearAxisAssembly::create(const Generic::LinearAxisConfig& config) {
    auto motorAssembly = MotorAssembly::create(config.motor);
    if (!motorAssembly) {
        return nullptr;
    }
    return create(std::move(motorAssembly), config);
}

std::unique_ptr<LinearAxisAssembly> LinearAxisAssembly::create(std::unique_ptr<MotorAssembly> motorAssembly,
                                                               const Generic::LinearAxisConfig& config) {
    if (config.distanceMmPerRevolution <= 0.0f) {
        LOG_ERROR("Invalid LinearAxisConfig: distanceMmPerRevolution must be > 0");
        return nullptr;
    }

    std::unique_ptr<LinearAxisAssembly> assembly(new LinearAxisAssembly());
    if (!assembly->initMotorAndEndstops(std::move(motorAssembly), config)) {
        return nullptr;
    }

    assembly->axis_ = std::make_unique<Generic::LinearAxis>(
        assembly->motorAssembly_->motor(),
        config.distanceMmPerRevolution);
    assembly->attachEndstopsAndLimits(config);

    return assembly;
}

// =============================================================================
// RotaryAxisAssembly
// =============================================================================

std::unique_ptr<RotaryAxisAssembly> RotaryAxisAssembly::create(const Generic::RotaryAxisConfig& config) {
    auto motorAssembly = MotorAssembly::create(config.motor);
    if (!motorAssembly) {
        return nullptr;
    }
    return create(std::move(motorAssembly), config);
}

std::unique_ptr<RotaryAxisAssembly> RotaryAxisAssembly::create(std::unique_ptr<MotorAssembly> motorAssembly,
                                                               const Generic::RotaryAxisConfig& config) {
    if (config.angleDegPerRevolution <= 0.0f) {
        LOG_ERROR("Invalid RotaryAxisConfig: angleDegPerRevolution must be > 0");
        return nullptr;
    }

    std::unique_ptr<RotaryAxisAssembly> assembly(new RotaryAxisAssembly());
    if (!assembly->initMotorAndEndstops(std::move(motorAssembly), config)) {
        return nullptr;
    }

    assembly->axis_ = std::make_unique<Generic::RotaryAxis>(
        assembly->motorAssembly_->motor(),
        config.angleDegPerRevolution,
        config.continuous);
    assembly->attachEndstopsAndLimits(config);

    return assembly;
}

} // namespace ESP32
} // namespace Stepper
