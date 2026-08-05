#include "StepperLog.hpp"
#include "StepperDriver_MCPWM.hpp"
#include "StepperGenerator.hpp"
//#include "FastRotaryEncoder.hpp"
#include <ESP32RotaryEncoder.h>


//FastRotaryEncoder encoder(33, 32, 35, 100, 100000, 100);
RotaryEncoder rotaryEncoder( 19, 18, 5, 21 );

//Stepper::Core core;
//Stepper::Task task;

Stepper::DriverMCPWM driver(25, 26, 27, 16);
Stepper::Generator generator(driver);
Stepper::Generator::GeneratorTask task;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  esp_log_level_set("*", ESP_LOG_INFO);

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
  task.steps = 0;
  task.velocity = 100000.0f;
  task.acceleration = 10000.0f;
  task.deceleration = 10000.0f;
  task.direction = Stepper::Direction::Clockwise;

  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
	rotaryEncoder.setBoundaries(0, 1000000, false);
  rotaryEncoder.setStepValue(1000);

  rotaryEncoder.onTurned([](long value) {
    Serial.print("Velocity: ");
    Serial.println(value);
    task.velocity = value;
  });

  rotaryEncoder.onPressed([](unsigned long duration) {
    Serial.println("Run task");
    generator.run(task);
  });
  rotaryEncoder.begin();

}

void loop() {
  //Serial.printf("CurrentVelocity: %f\n", generator.getVelocity());
  //Serial.printf("Steps: %lu\n", static_cast<uint32_t>(driver.getSteps()));
  vTaskDelay(pdMS_TO_TICKS(5000));
}
