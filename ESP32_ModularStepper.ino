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

Stepper::DriverMCPWM driver(26, 25, 27);
Stepper::Generator generator(driver);
Stepper::Motor motor(generator);
Stepper::RotaryAxis axis(motor);

volatile float velocityDegPerSecond = 1000.0f;

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
  motor.setParams(200, 1.0f);
  axis.setZero(0.0f);

  rotaryEncoder.setEncoderType( EncoderType::HAS_PULLUP );
	rotaryEncoder.setBoundaries(0, 1000000, false);
  rotaryEncoder.setStepValue(1000);

  rotaryEncoder.onTurned([](long value) {
    Serial.print("Velocity: ");
    Serial.println(value);
    velocityDegPerSecond = static_cast<float>(value);
  });

  rotaryEncoder.onPressed([](unsigned long duration) {
    if (duration < 400) {
      Serial.println("moveBy(90 deg)");
      axis.moveBy(90.0f, velocityDegPerSecond, 720.0f, 720.0f);
    } else {
      Serial.println("setVelocity() 2s -> stop()");
      axis.setVelocity(velocityDegPerSecond, 720.0f, 720.0f);
      vTaskDelay(pdMS_TO_TICKS(2000));
      axis.stop(720.0f);
    }
  });
  rotaryEncoder.begin();

}

void loop() {
  axis.update();
  Serial.printf("Axis pos(deg): %.2f vel(deg/s): %.2f state: %u\n",
                axis.getPosition(),
                axis.getVelocity(),
                static_cast<uint8_t>(axis.getState()));
  vTaskDelay(pdMS_TO_TICKS(250));
}
