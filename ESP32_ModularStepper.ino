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

Stepper::DriverMCPWM driver(25, 26, 27);
Stepper::Generator generator(driver);
Stepper::Motor motor(generator);

Stepper::Generator::GeneratorTask task;

volatile float velocity = 1.0f;
float acceleration = 10.0f;
float deceleration = 10.0f;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  esp_log_level_set("*", ESP_LOG_INFO);

  while (!Serial) { vTaskDelay(pdMS_TO_TICKS(1000)); }

  task.steps = 0;
  task.velocity = 1000.0f;
  task.acceleration = 1000.0f;
  task.deceleration = 1000.0f;
  task.direction = Stepper::Direction::Counterclockwise;

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

  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
	rotaryEncoder.setBoundaries(0, 1000, false);
  rotaryEncoder.setStepValue(1);

  rotaryEncoder.onTurned([](long value) {
    Serial.print("Velocity: ");
    Serial.println(value);
    velocity = static_cast<float>(value);
  });

  rotaryEncoder.onPressed([](unsigned long duration) {
    if (duration < 400) {
      Serial.println("Move by 90 degrees");
      motor.run(90.0f, velocity, acceleration, deceleration, Stepper::Direction::Clockwise);
    } else {
      if (motor.getVelocity() > 0.0f) {
        Serial.println("Stopping motor");
        motor.run(0.0f, acceleration, deceleration, Stepper::Direction::Clockwise);
      } else {
        
        if (generator.run(task)) {
          Serial.println("Run motor at constant velocity");
        }
        else {
          Serial.println("Failed to run motor");
        }
        //motor.run(velocity, acceleration, deceleration, Stepper::Direction::Clockwise);
      }
    }
  });
  rotaryEncoder.begin();

  driver.enable();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
