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

// Endstop pins on GPIO 12 (Min) & GPIO 14 (Max), active-low (inverse=true) with internal pull-up
Stepper::ESP32::Pin pinEndstopMin(12, true, Stepper::Generic::Pin::IoMode::Input, Stepper::Generic::Pin::PullMode::PullUp);
Stepper::ESP32::Pin pinEndstopMax(14, true, Stepper::Generic::Pin::IoMode::Input, Stepper::Generic::Pin::PullMode::PullUp);
Stepper::ESP32::Endstop endstopMin(pinEndstopMin);
Stepper::ESP32::Endstop endstopMax(pinEndstopMax);

Stepper::ESP32::MCPWM::Driver driver(pinEnable, pinStep, pinDirection, 16);
Stepper::Generic::Generator generator(driver);
Stepper::Generic::Motor motor(generator, 200, 1.0f);
Stepper::Generic::RotaryAxis axis(motor, 360.0f, false);
Stepper::Generic::RotaryAxis::RotaryAxisTask task;
Stepper::ESP32::Homing homing;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  LOG_LEVEL_INIT();

  while (!Serial) { vTaskDelay(pdMS_TO_TICKS(1000)); }

  // Configure endstop debounce (5 ms) and attach to rotary axis
  endstopMin.setDebounceUs(5000);
  endstopMax.setDebounceUs(5000);
  axis.setEndstops(&endstopMin, &endstopMax);

  task.angle = 90.0f;
  task.velocity = 1.0;
  task.acceleration = 100.0f;
  task.deceleration = 100.0f;
  task.moveRelative = true;

  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
	rotaryEncoder.setBoundaries(0, 360, false);
  rotaryEncoder.setStepValue(1);
  rotaryEncoder.setEncoderValue(task.angle);
  // rotaryEncoder.setEncoderValue(task.velocity);

  // Initialize dual-endstop homing (Min -> Max) with very slow speeds for safe testing
  Stepper::Generic::Homing::Config homingCfg;
  homingCfg.strategy = Stepper::Generic::Homing::Strategy::MinThenMax;
  homingCfg.fastVelocity = 10.0f;      // 10 deg/s
  homingCfg.slowVelocity = 2.0f;       // 2 deg/s
  homingCfg.acceleration = 20.0f;      // 20 deg/s^2
  homingCfg.deceleration = 20.0f;      // 20 deg/s^2
  homingCfg.retractDistance = 5.0f;    // 5 deg back-off between fast & slow approach
  homingCfg.backoffDistance = 5.0f;    // 5 deg final pull-off so motor doesn't rest on Max switch
  homingCfg.maxTravelDistance = 270.0f; // abort if switch not found within 270 deg
  homingCfg.homePosition = 0.0f;
  homing.setConfig(homingCfg);

  rotaryEncoder.onTurned([](long value) {
    Serial.print("Angle: ");
    Serial.println(value);
    task.angle = value;
  });

  rotaryEncoder.onPressed([](unsigned long duration) {
    Serial.println("Run task");
    if (duration > 500) {
      homing.homeMinMaxAsync(axis, [](bool success) {
      Serial.printf("Dual-endstop homing finished: %s | Limits: [%.2f, %.2f] deg | Angle: %.2f deg\n",
                  success ? "SUCCESS" : "FAILED",
                  axis.getMinAngleDeg(),
                  axis.getMaxAngleDeg(),
                  axis.getAngleDeg());
      });
    }
    axis.run(task);
  });

  rotaryEncoder.begin();
  driver.enable();
}

void loop() {
  Serial.printf("Min(22): %s | Max(23): %s | Homing: %d | Homed: %d | Angle: %.2f deg | Limits: [%.2f, %.2f]\n",
                endstopMin.isTriggered() ? "TRIG" : "open",
                endstopMax.isTriggered() ? "TRIG" : "open",
                static_cast<int>(homing.getState()),
                axis.isHomed(),
                axis.getAngleDeg(),
                axis.getMinAngleDeg(),
                axis.getMaxAngleDeg());
  vTaskDelay(pdMS_TO_TICKS(500));
}
