#include <ESP32_ModularStepper.h>
#include <memory>

Stepper::Generic::StopController stopController;
std::unique_ptr<Stepper::ESP32::LinearAxisAssembly> axisAssembly;

void setup() {
  Serial.begin(115200);

  Stepper::Generic::LinearAxisConfig axisCfg;
  axisCfg.id = 0; // Auto-register with StopController
  axisCfg.distanceMmPerRevolution = 5.0f;
  axisCfg.fastStopDeceleration = 100.0f; // mm/s^2

  axisCfg.motor.driver.type = Stepper::Generic::DriverConfig::Type::MCPWM;
  axisCfg.motor.driver.enablePin    = {25, false, Stepper::Generic::Pin::IoMode::Output, Stepper::Generic::Pin::PullMode::Floating};
  axisCfg.motor.driver.stepPin      = {26, false, Stepper::Generic::Pin::IoMode::Output, Stepper::Generic::Pin::PullMode::Floating};
  axisCfg.motor.driver.directionPin = {27, false, Stepper::Generic::Pin::IoMode::Output, Stepper::Generic::Pin::PullMode::Floating};
  axisCfg.motor.driver.microsteps   = 16;

  axisCfg.motor.stepsPerRevolution = 200;
  axisCfg.motor.gearRatio = 1.0f;

  axisAssembly = Stepper::ESP32::LinearAxisAssembly::create(axisCfg, &stopController);
  if (!axisAssembly) {
    Serial.println("Failed to create LinearAxisAssembly");
    return;
  }

  axisAssembly->driver().enable();

  // Move +25 mm at 10 mm/s with 20 mm/s^2 accel/decel
  axisAssembly->axis().run(25.0f, 10.0f, 20.0f, 20.0f);
  delay(500);

  // Pause and resume via StopController
  if (stopController.pause(0)) {
    axisAssembly->axis().waitForState(Stepper::State::Paused);
    Serial.printf("Paused at %.2f mm\n", axisAssembly->axis().getPositionMm());
    delay(500);
    stopController.resume(0);
  }

  axisAssembly->axis().waitForState(Stepper::State::Stopped);
  Serial.printf("Completed move at %.2f mm\n", axisAssembly->axis().getPositionMm());

  // Demonstrate fastStop and emergencyStop latch
  axisAssembly->axis().run(-25.0f, 10.0f, 20.0f, 20.0f);
  delay(200);
  stopController.fastStop(0);
  axisAssembly->axis().waitForState(Stepper::State::Stopped);

  stopController.emergencyStopAll(true);
  stopController.clearEmergencyStopAll(true);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(500));
}
