#include <ESP32RotaryEncoder.h>
//#include "FastRotaryEncoder.hpp"

#include "src/Generic/Log.hpp"
#include "src/ESP32/Pin.hpp"
#include "src/ESP32/Driver_MCPWM.hpp"
#include "src/Generic/Generator.hpp"
#include "src/Generic/Motor.hpp"
#include "src/ESP32/Endstop.hpp"
#include "src/Generic/Axis_Rotary.hpp"
#include "src/ESP32/Homing.hpp"


//FastRotaryEncoder encoder(33, 32, 35, 100, 100000, 100);
RotaryEncoder rotaryEncoder( 19, 18, 5, 21 );

//Stepper::Core core;
//Stepper::Task task;
Stepper::ESP32::Pin pinEnable(25);
Stepper::ESP32::Pin pinStep(26);
Stepper::ESP32::Pin pinDirection(27);

// Endstop pins on GPIO 22 (Min) & GPIO 23 (Max), active-low (inverse=true) with internal pull-up
Stepper::ESP32::Pin pinEndstopMin(22, true, Stepper::Generic::Pin::IoMode::Input, Stepper::Generic::Pin::PullMode::PullUp);
Stepper::ESP32::Pin pinEndstopMax(23, true, Stepper::Generic::Pin::IoMode::Input, Stepper::Generic::Pin::PullMode::PullUp);
Stepper::ESP32::Endstop endstopMin(pinEndstopMin);
Stepper::ESP32::Endstop endstopMax(pinEndstopMax);

Stepper::ESP32::MCPWM::Driver driver(pinEnable, pinStep, pinDirection, 16);
Stepper::Generic::Generator generator(driver);
Stepper::Generic::Motor motor(generator, 200, 1.0f);
Stepper::Generic::RotaryAxis axis(motor, 360.0f, false);
Stepper::ESP32::Homing homing;

Stepper::Generic::Motor::MotorTask task;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  LOG_LEVEL_INIT();

  while (!Serial) { vTaskDelay(pdMS_TO_TICKS(1000)); }

/*
  encoder.setup();
  encoder.initMappedPosition(0); // reset init value

  // lambda, register callback when mappedPosition value changed
  encoder.registerEncoderChangedCb([](long mappedPosition) {
      Serial.print("Encoder1: ");
      Serial.println(mappedPosition);
  });

  encoder.getButton()->attachClick([]() {
      Serial.println("Encoder1: click");
  });
*/

  task.velocity = 1.0;
  task.revolutions = 90.0f;
  task.acceleration = 100.0f;
  task.deceleration = 100.0f;
  task.direction = Stepper::Direction::Clockwise;

  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
	rotaryEncoder.setBoundaries(0, 1'000'000, false);
  rotaryEncoder.setStepValue(1);
  rotaryEncoder.setEncoderValue(task.revolutions);
  // rotaryEncoder.setEncoderValue(task.velocity.getInteger());

  rotaryEncoder.onTurned([](long value) {
    // Serial.print("Steps: ");
    // Serial.println(value);
    // task.steps = value;

    Serial.print("Angle: ");
    Serial.println(value);
    task.revolutions = value;
  });

  rotaryEncoder.onPressed([](unsigned long duration) {
    Serial.println("Run task");
    if (duration > 500) {
      Serial.println("Change direction");
      if (task.direction == Stepper::Direction::Clockwise)
        task.direction = Stepper::Direction::Counterclockwise;
      else
        task.direction = Stepper::Direction::Clockwise;
    }
    motor.run(task);
  });
  rotaryEncoder.begin();
  driver.enable();

  // Configure endstop debounce (5 ms) and attach to rotary axis
  endstopMin.setDebounceUs(5000);
  endstopMax.setDebounceUs(5000);
  axis.setEndstops(&endstopMin, &endstopMax);

  // Initialize homing with very slow speeds for safe first testing
  Stepper::Generic::Homing::Config homingCfg;
  homingCfg.endstop = Stepper::Generic::Axis::EndstopPosition::Min;
  homingCfg.fastVelocity = 10.0f;      // 10 deg/s
  homingCfg.slowVelocity = 2.0f;       // 2 deg/s
  homingCfg.acceleration = 20.0f;      // 20 deg/s^2
  homingCfg.deceleration = 20.0f;      // 20 deg/s^2
  homingCfg.retractDistance = 5.0f;    // 5 deg back-off between fast & slow approach
  homingCfg.backoffDistance = 90.0f;    // 90 deg final pull-off so motor doesn't rest on switch
  homingCfg.homePosition = 0.0f;
  homingCfg.timeoutMs = 30000;         // 30 s timeout
  homing.setConfig(homingCfg);

  // Uncomment to start async homing on startup:
  homing.homeAsync(axis, [](bool success) {
    Serial.printf("Homing finished: %s\n", success ? "SUCCESS" : "FAILED");
  });
}

void loop() {
  Serial.printf("Endstop Min (GPIO 22): %s (lvl=%d) | Endstop Max (GPIO 23): %s (lvl=%d) | Homing: %d | Homed: %d | Angle: %.2f deg\n",
                endstopMin.isTriggered() ? "TRIGGERED" : "open",
                endstopMin.getLevel(),
                endstopMax.isTriggered() ? "TRIGGERED" : "open",
                endstopMax.getLevel(),
                static_cast<int>(homing.getState()),
                axis.isHomed(),
                axis.getAngleDeg());
  vTaskDelay(pdMS_TO_TICKS(500));
}
