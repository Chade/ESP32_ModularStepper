#include <ESP32RotaryEncoder.h>
#include <ESP32_ModularStepper.h>

#include <memory>

RotaryEncoder rotaryEncoder( 19, 18, 5, 21 );

std::unique_ptr<Stepper::ESP32::LinearAxisAssembly> axisAssembly;
Stepper::Generic::LinearAxis::LinearAxisTask task;
Stepper::ESP32::Homing homing;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  LOG_LEVEL_INIT();

  while (!Serial) { vTaskDelay(pdMS_TO_TICKS(1000)); }

  // Build runtime configuration for the linear axis
  Stepper::Generic::LinearAxisConfig axisCfg;
  axisCfg.distanceMmPerRevolution = 5.0f;

  axisCfg.motor.driver.type = Stepper::Generic::DriverConfig::Type::MCPWM;
  axisCfg.motor.driver.enablePin    = {25, false, Stepper::Generic::Pin::IoMode::Input_Output, Stepper::Generic::Pin::PullMode::Floating};
  axisCfg.motor.driver.stepPin      = {26, false, Stepper::Generic::Pin::IoMode::Input_Output, Stepper::Generic::Pin::PullMode::Floating};
  axisCfg.motor.driver.directionPin = {27, false, Stepper::Generic::Pin::IoMode::Input_Output, Stepper::Generic::Pin::PullMode::Floating};
  axisCfg.motor.driver.microsteps   = 16;

  axisCfg.motor.stepsPerRevolution = 200;
  axisCfg.motor.gearRatio = 1.0f;

  // Endstop pins on GPIO 12 (Min) & GPIO 14 (Max) with internal pull-up
  axisCfg.endstopMin.enabled = true;
  axisCfg.endstopMin.pin = {12, false, Stepper::Generic::Pin::IoMode::Input, Stepper::Generic::Pin::PullMode::PullUp};
  axisCfg.endstopMin.debounceUs = 100;
  axisCfg.endstopMin.limitProtection = true;

  axisCfg.endstopMax.enabled = true;
  axisCfg.endstopMax.pin = {14, false, Stepper::Generic::Pin::IoMode::Input, Stepper::Generic::Pin::PullMode::PullUp};
  axisCfg.endstopMax.debounceUs = 100;
  axisCfg.endstopMax.limitProtection = true;

  axisAssembly = Stepper::ESP32::LinearAxisAssembly::create(axisCfg);
  if (!axisAssembly) {
    LOG_ERROR("Failed to initialize LinearAxisAssembly");
    return;
  }

  task.position = 0.0f;
  task.velocity = 10.0;
  task.acceleration = 10.0f;
  task.deceleration = 10.0f;

  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
  rotaryEncoder.setBoundaries(-100, 100, false);
  rotaryEncoder.setStepValue(1);
  rotaryEncoder.setEncoderValue(task.position);

  // Initialize dual-endstop homing (Min -> Max)
  Stepper::Generic::Homing::Config homingCfg;
  homingCfg.strategy = Stepper::Generic::Homing::Strategy::MinThenMax;
  homingCfg.fastVelocity = 10.0f;
  homingCfg.slowVelocity = 2.0f;
  homingCfg.acceleration = 50.0f;
  homingCfg.deceleration = 50.0f;
  homingCfg.retractDistance = 5.0f;
  homingCfg.backoffDistance = 10.0f;
  homingCfg.maxTravelDistance = 115.0f;
  homingCfg.homePosition = 0.0f;
  homing.setConfig(homingCfg);

  rotaryEncoder.onTurned([](long value) {
    Serial.print("Position: ");
    Serial.println(value);
    task.position = value;
  });

  rotaryEncoder.onPressed([](unsigned long duration) {
    if (!axisAssembly) {
      return;
    }
    auto& axis = axisAssembly->axis();
    Serial.println("Run task");
    if (duration > 500) {
      homing.homeMinMaxAsync(axis, [&axis](bool success) {
        Serial.printf("Dual-endstop homing finished: %s | Limits: [%.2f, %.2f] mm | Position: %.2f mm\n",
                      success ? "SUCCESS" : "FAILED",
                      axis.getMinPositionMm(),
                      axis.getMaxPositionMm(),
                      axis.getPosition());
      });
      return;
    }
    axis.run(task);
  });

  rotaryEncoder.begin();
  axisAssembly->driver().enable();
}

void loop() {
  Serial.printf("Min(12): %s | Max(14): %s | Homing: %d | Homed: %d | Position: %.2f mm | Limits: [%.2f, %.2f]\n",
                axisAssembly->endstopMin()->isTriggered() ? "TRIG" : "open",
                axisAssembly->endstopMax()->isTriggered() ? "TRIG" : "open",
                static_cast<int>(homing.getState()),
                axisAssembly->axis().isHomed(),
                axisAssembly->axis().getPosition(),
                axisAssembly->axis().getMinPositionMm(),
                axisAssembly->axis().getMaxPositionMm());
  vTaskDelay(pdMS_TO_TICKS(500));
}
