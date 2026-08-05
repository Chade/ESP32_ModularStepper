#include "StepperLog.hpp"
#include "StepperDriver_MCPWM.hpp"
#include "StepperGenerator.hpp"
#include "StepperMotor.hpp"
#include "StepperAxis.hpp"
//#include "FastRotaryEncoder.h"
#include <ESP32RotaryEncoder.h>


//FastRotaryEncoder encoder(33, 32, 35, 100, 100000, 100);
RotaryEncoder rotaryEncoder( 19, 18, 5, 21 );

//Stepper::Core core;
//Stepper::Task task;

Stepper::DriverMCPWM driver(25, 26, 27, 16);
Stepper::Generator generator(driver);
Stepper::Motor motor(generator);

volatile float velocity = 0.1f;
float acceleration = 1.0f;
float deceleration = 1.0f;
uint8_t steps = 0;
Stepper::Generator::GeneratorTask task;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  esp_log_level_set("*", ESP_LOG_INFO);

  while (!Serial) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  
  task.steps = 0;
  task.velocity = 100000.0f;
  task.acceleration = 10000.0f;
  task.deceleration = 10000.0f;
  task.direction = Stepper::Direction::Clockwise;
  
  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
	rotaryEncoder.setBoundaries(0, 1'000'000, false);
  rotaryEncoder.setStepValue(1000);

  rotaryEncoder.onTurned([](long value) {
    Serial.print("Velocity: ");
    Serial.println(value);
    task.velocity = value;
  });

  rotaryEncoder.onPressed([](unsigned long duration) {
    generator.run(task);
    if (duration < 400) {
      //motor.run(360.0f, velocity, acceleration, deceleration, Stepper::Direction::Clockwise);
    } else {
      generator.run(task);
      //motor.run(velocity, acceleration, deceleration, Stepper::Direction::Clockwise);
    }
  });
  rotaryEncoder.begin();

  driver.enable();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(5000));
}
