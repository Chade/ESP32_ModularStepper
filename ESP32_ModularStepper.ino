#include <ESP32RotaryEncoder.h>
//#include "FastRotaryEncoder.hpp"

#include "StepperLog.hpp"
#include "StepperPin_ESP32.hpp"
#include "StepperDriver_ESP32_MCPWM.hpp"
#include "StepperGenerator.hpp"
#include "StepperMotor.hpp"


//FastRotaryEncoder encoder(33, 32, 35, 100, 100000, 100);
RotaryEncoder rotaryEncoder( 19, 18, 5, 21 );

//Stepper::Core core;
//Stepper::Task task;
Stepper::ESP32::Pin pinEnable(25);
Stepper::ESP32::Pin pinStep(26);
Stepper::ESP32::Pin pinDirection(27);

Stepper::ESP32::MCPWM::Driver driver(pinEnable, pinStep, pinDirection, 16);
Stepper::Generic::Generator generator(driver);
Stepper::Generic::Motor motor(generator, 200, 1.0f);

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

}

void loop() {
  //Serial.printf("CurrentVelocity: %f\n", generator.getVelocity());
  //Serial.printf("Steps: %lu\n", static_cast<uint32_t>(driver.getSteps()));
  vTaskDelay(pdMS_TO_TICKS(5000));
}
