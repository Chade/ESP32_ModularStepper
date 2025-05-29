#include "StepperCore.h"

Stepper::Core core;
Stepper::Task task;

void setup() {
  // put your setup code here, to run once:
  core.start();
}

void loop() {
  ++task.id;

  esp_err_t err = Stepper::taskEventLoopPost(Stepper::TaskEventId::NEW_TASK_EVENT, &task, sizeof(struct Stepper::Task), 1000);
  ESP_LOGE("Main", "Posting event %s", esp_err_to_name(err));
  
  vTaskDelay(pdMS_TO_TICKS(5000));
}
