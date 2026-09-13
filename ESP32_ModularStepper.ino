#include <ESP32RotaryEncoder.h>
//#include "FastRotaryEncoder.hpp"

#include "StepperLog.hpp"
#include "StepperPin_ESP32.hpp"
#include "StepperDriver_ESP32_MCPWM.hpp"
#include "StepperGenerator.hpp"


//FastRotaryEncoder encoder(33, 32, 35, 100, 100000, 100);
RotaryEncoder rotaryEncoder( 19, 18, 5, 21 );

//Stepper::Core core;
//Stepper::Task task;
Stepper::ESP32::Pin pinEnable(25);
Stepper::ESP32::Pin pinStep(26);
Stepper::ESP32::Pin pinDirection(27);

Stepper::ESP32::MCPWM::Driver driver(pinEnable, pinStep, pinDirection, 16);
Stepper::Generic::Generator generator(driver);
Stepper::Generic::Generator::GeneratorTask task;

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
  task.steps = 3200;
  // task.steps = 0;
  task.velocity = 1000.0f;
  task.acceleration = 0.0f;
  task.deceleration = 1000.0f;
  task.direction = Stepper::Direction::Clockwise;

  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
	rotaryEncoder.setBoundaries(0, 1'000'000, false);
  rotaryEncoder.setStepValue(100);
  rotaryEncoder.setEncoderValue(task.steps);
  // rotaryEncoder.setEncoderValue(task.velocity.getInteger());

  rotaryEncoder.onTurned([](long value) {
    // Serial.print("Steps: ");
    // Serial.println(value);
    // task.steps = value;

    Serial.print("Velocity: ");
    Serial.println(value);
    task.velocity = value;
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
    generator.run(task);
  });
  rotaryEncoder.begin();
  driver.enable();

}

void loop() {
  //Serial.printf("CurrentVelocity: %f\n", generator.getVelocity());
  //Serial.printf("Steps: %lu\n", static_cast<uint32_t>(driver.getSteps()));
  vTaskDelay(pdMS_TO_TICKS(5000));
}
