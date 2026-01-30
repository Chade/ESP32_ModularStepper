#include "StepperGenerator.h"
#include "StepperLog.h"

namespace Stepper {

    Generator::Generator(Driver& driver) : m_driver(driver) {
        esp_log_level_set(log_tag, ESP_LOG_INFO);

        // Create timer
        createTimer(TimerType::Hardware);

        // Create generator task queue
        m_taskQueueHandle = xQueueCreate(k_taskQueueSize, sizeof(struct GeneratorTask));
        assert(m_taskQueueHandle != 0);

        // Create stepper task queue
        m_stepQueueHandle = xQueueCreate(2, sizeof(struct StepperTask));
        assert(m_stepQueueHandle != 0);
    }

    Generator::~Generator() {
        deleteTimer();

        if (m_taskQueueHandle) {
            vQueueDelete(m_taskQueueHandle);
            m_taskQueueHandle = nullptr;
        }

        if (m_stepQueueHandle) {
            vQueueDelete(m_stepQueueHandle);
            m_stepQueueHandle = nullptr;
        }
    }

    void Generator::createTimer(Generator::TimerType type) {
        if (type == Generator::TimerType::Software) {
            ESP_LOGI(log_tag, "Create software timer");
            esp_timer_create_args_t args{};
            args.callback = &Generator::timerCallback;
            args.arg = this;
            args.dispatch_method = ESP_TIMER_TASK;
            args.name = "StepperGeneratorTimer";
            ESP_ERROR_CHECK(esp_timer_create(&args, &m_timer));
        }
        else if (type == Generator::TimerType::Hardware) {
            ESP_LOGI(log_tag, "Create hardware timer");
            gptimer_config_t timer_config;
            timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
            timer_config.direction = GPTIMER_COUNT_UP;
            timer_config.resolution_hz = k_timerResolutionHz; // 1 MHz
            timer_config.intr_priority = 3;
            timer_config.flags.intr_shared = 0;
            timer_config.flags.allow_pd = 0;
            timer_config.flags.backup_before_sleep = 0;
            ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &m_gptimer));

            gptimer_event_callbacks_t callback_config;
            callback_config.on_alarm = &Generator::gptimerOnAlarm;
            ESP_ERROR_CHECK(gptimer_register_event_callbacks(m_gptimer, &callback_config, this));

            ESP_ERROR_CHECK(gptimer_enable(m_gptimer));
        }
        ESP_LOGI(log_tag, "Timer created");
    }

    void Generator::rearmTimer(uint64_t period_us) {
        if (m_timer) {
            if (esp_timer_is_active(m_timer)) {
                ESP_ERROR_CHECK(esp_timer_stop(m_timer));
            }
            ESP_ERROR_CHECK(esp_timer_start_once(m_timer, period_us));
        }

        if (m_gptimer) {
            ESP_ERROR_CHECK(gptimer_stop(m_gptimer));
            ESP_ERROR_CHECK(gptimer_set_raw_count(m_gptimer, 0));
            

            gptimer_alarm_config_t alarm_config;
            alarm_config.alarm_count = 30;
            alarm_config.reload_count = 0;
            alarm_config.flags.auto_reload_on_alarm = true;

            ESP_ERROR_CHECK(gptimer_set_alarm_action(m_gptimer, &alarm_config));
            ESP_ERROR_CHECK(gptimer_start(m_gptimer));
        }
    }

    void Generator::stopTimer() {
        if (m_timer) {
            if (esp_timer_is_active(m_timer)) {
                ESP_ERROR_CHECK(esp_timer_stop(m_timer));
            }
        }

        if (m_gptimer) {
            ESP_ERROR_CHECK(gptimer_stop(m_gptimer));
            ESP_ERROR_CHECK(gptimer_set_raw_count(m_gptimer, 0));
        }
        ESP_LOGI(log_tag, "Timer stopped");
    }

    void Generator::deleteTimer() {
        if (m_timer) {
            esp_timer_stop(m_timer);
            esp_timer_delete(m_timer);
            m_timer = nullptr;
        }

        if (m_gptimer) {
            gptimer_stop(m_gptimer);
            gptimer_disable(m_gptimer);
            gptimer_del_timer(m_gptimer);
            m_gptimer = nullptr;
        }
    }


    void Generator::start() {
        // Create task
        if (m_taskHandle == nullptr)
        {
            xTaskCreatePinnedToCore(
                Generator::generatorTask,    /* Task function */
                "GeneratorTask",             /* Task name (max. 16 characters by default) */
                2048,                        /* Stack size in bytes */
                this,                        /* Parameter passed as input of the task */
                10,                          /* Task priority */
                &m_taskHandle,               /* Task handle */
                0                            /* CPU core to use */
            );
        }
        else
        {
            ESP_LOGE(log_tag, "Task already started!");
        }
        ESP_LOGI(log_tag, "Started");
    }

    void Generator::stop() {
        // Delete timer
        stopTimer();

        // Reset internal state
        resetState();

        // Delete task
        if (m_taskHandle != nullptr)
        {
            vTaskDelete(m_taskHandle);
            m_taskHandle = nullptr;
        }
        else
        {
            ESP_LOGE(log_tag, "Task already stopped!");
        }

        // Reset queues
        xQueueReset(m_taskQueueHandle);
        xQueueReset(m_stepQueueHandle);

        ESP_LOGI(log_tag, "Stopped");
    }

    bool Generator::run(const GeneratorTask& task, uint32_t waitFor_ms) {
        return xQueueSend(m_taskQueueHandle, (void*)&task, pdMS_TO_TICKS(waitFor_ms));
    }

    bool Generator::run(float targetVelocity,
                        float acceleration,
                        float deceleration,
                        Direction direction,
                        uint32_t waitFor_ms) {
        GeneratorTask task;

        // Clamp to non-negative values
        task.steps = 0;
        task.velocity = std::max(0.0f, targetVelocity);
        task.acceleration = std::max(0.0f, acceleration);
        task.deceleration = std::max(0.0f, deceleration);
        task.direction = direction;

        return run(task, waitFor_ms);
    }

/*
    void Generator::setVelocity(float targetVelocity, float acceleration, float deceleration, Direction direction) {
        ESP_LOGI(log_tag, "Set velocity");
        ESP_LOGI(log_tag, "vel=%.1f acc=%.1f dec=%.1f dir=%i", targetVelocity, acceleration, deceleration, cast_enum_to_base(direction));
        // Clamp to non-negative values

        m_state.targetVelocity = std::max(0.0f, targetVelocity);
        m_state.acceleration   = std::max(0.0f, acceleration);
        m_state.deceleration   = std::max(0.0f, deceleration);
        m_state.targetDirection = direction;

        if (m_state.currentVelocity <= 0.0f && m_state.targetVelocity > 0.0f) {
            //m_state.currentVelocity = std::min(k_minVelocity, m_state.targetVelocity); // seed to avoid div by zero
            m_state.currentVelocity = k_minVelocity; // seed to avoid div by zero
        
            m_driver.setDirection(m_state.targetDirection);
            m_state.currentDirection = m_state.targetDirection;
        }

        if (!compare_enums(m_state.state, State::Running)) {
            uint64_t period = computeStepPeriodUs(m_state.currentVelocity);
            m_state.state = State::Running;
            scheduleNextStep(period);
            ESP_LOGI(log_tag, "Scheduled first step with period %i us @ %.2f steps/s", period, m_state.currentVelocity);
        }

        ESP_LOGI(log_tag, "Current velocity: %.2f steps/s    New velocity: %.2f steps/s", m_state.currentVelocity, m_state.targetVelocity);
    }
*/

    bool Generator::run(uint64_t steps,
                              float targetVelocity,
                              float acceleration,
                              float deceleration,
                              Direction direction,
                              uint32_t waitFor_ms) {
        GeneratorTask task;

        // Clamp to non-negative values
        task.steps = steps;
        task.velocity = std::max(0.0f, targetVelocity);
        task.acceleration = std::max(0.0f, acceleration);
        task.deceleration = std::max(0.0f, deceleration);
        task.direction = direction;

        return run(task, waitFor_ms);
    }

/*
    void Generator::moveSteps(uint64_t steps,
                              float targetVelocity,
                              float acceleration,
                              float deceleration,
                              Direction direction) {
        ESP_LOGI(log_tag, "Set steps");
        ESP_LOGI(log_tag, "Move %i steps", steps);
        // Clamp inputs
        targetVelocity = std::max(0.0f, targetVelocity);
        acceleration   = std::max(0.0f, acceleration);
        deceleration   = std::max(0.0f, deceleration);

        m_state.stepsTotal = steps;
        m_state.stepsDone = 0;
        
        // Compute ramp distribution (start from currentVelocity)
        // Steps to accelerate from currentVelocity to targetVelocity
        auto calcStepsNeeded = [](float v0, float v1, float a) -> uint64_t {
            if (a <= 0.0f) return 0;
            float dv = std::max(0.0f, v1 - v0);
            // s = 0.5 * dv^2 / a
            float s = 0.5f * dv * dv / a;
            return static_cast<uint32_t>(s);
        };
        
        uint32_t stepsAcc = calcStepsNeeded(m_state.currentVelocity, targetVelocity, acceleration);
        uint32_t stepsDec = calcStepsNeeded(targetVelocity, 0.0f, deceleration);

        if (stepsAcc + stepsDec <= m_state.stepsTotal) {
            m_state.stepsAcc   = stepsAcc;
            m_state.stepsDec   = stepsDec;
            m_state.stepsConst = m_state.stepsTotal - (stepsAcc + stepsDec);
        } else {
            // Triangular profile: scale acc/dec phases to fit total steps
            if (stepsAcc == 0 && stepsDec != 0) {
                m_state.stepsAcc = 0; m_state.stepsConst = 0; m_state.stepsDec = m_state.stepsTotal;
            } else if (stepsAcc != 0 && stepsDec == 0) {
                m_state.stepsAcc = m_state.stepsTotal; m_state.stepsConst = 0; m_state.stepsDec = 0;
            } else if (stepsAcc == 0 && stepsDec == 0) {
                m_state.stepsAcc = 0; m_state.stepsConst = m_state.stepsTotal; m_state.stepsDec = 0;
            } else {
                float ratio = std::min(stepsAcc, stepsDec) / static_cast<float>(std::max(stepsAcc, stepsDec));
                m_state.stepsAcc = static_cast<uint64_t>(m_state.stepsTotal * ratio);
                m_state.stepsDec = m_state.stepsTotal - m_state.stepsAcc;
                m_state.stepsConst = 0;
            }
        }

        setVelocity(targetVelocity, acceleration, deceleration, direction);
    }
*/

    bool Generator::initializeStateBeforeStep(const GeneratorTask& task, GeneratorState& state) {
        state.targetVelocity  = std::max(0.0f, task.velocity);
        state.acceleration    = std::max(0.0f, task.acceleration);
        state.deceleration    = std::max(0.0f, task.deceleration);
        state.targetDirection = task.direction;
        state.stepsTotal      = task.steps;
        state.stepsDone       = 0;

        bool ret = false;

        // Check if we are starting from stand-still
        if (state.currentVelocity <= 0.0f && state.targetVelocity > 0.0f) {
            state.currentVelocity = k_minVelocity; // seed to avoid div by zero
            ret = true;
        }

        if (state.stepsTotal > 0) {
            
            // Compute ramp distribution (start from currentVelocity)
            // Steps to accelerate from currentVelocity to targetVelocity
            auto calcStepsNeeded = [](float v0, float v1, float a) -> uint64_t {
                if (a <= 0.0f) return 0;
                float dv = std::max(0.0f, v1 - v0);
                // s = 0.5 * dv^2 / a
                float s = 0.5f * dv * dv / a;
                return static_cast<uint32_t>(s);
            };
            
            uint32_t stepsAcc = calcStepsNeeded(state.currentVelocity, state.targetVelocity, state.acceleration);
            uint32_t stepsDec = calcStepsNeeded(state.targetVelocity, 0.0f, state.deceleration);

            if (stepsAcc + stepsDec <= state.stepsTotal) {
                state.stepsAcc   = stepsAcc;
                state.stepsDec   = stepsDec;
                state.stepsConst = state.stepsTotal - (stepsAcc + stepsDec);
            } else {
                // Triangular profile: scale acc/dec phases to fit total steps
                if (stepsAcc == 0 && stepsDec != 0) {
                    state.stepsAcc = 0;
                    state.stepsConst = 0;
                    state.stepsDec = state.stepsTotal;
                } else if (stepsAcc != 0 && stepsDec == 0) {
                    state.stepsAcc = state.stepsTotal;
                    state.stepsConst = 0;
                    state.stepsDec = 0;
                } else if (stepsAcc == 0 && stepsDec == 0) {
                    state.stepsAcc = 0;
                    state.stepsConst = state.stepsTotal;
                    state.stepsDec = 0;
                } else {
                    float ratio = std::min(stepsAcc, stepsDec) / static_cast<float>(std::max(stepsAcc, stepsDec));
                    state.stepsAcc = static_cast<uint64_t>(state.stepsTotal * ratio);
                    state.stepsDec = state.stepsTotal - state.stepsAcc;
                    state.stepsConst = 0;
                }
            }
        }
        return ret;
    }

    State Generator::getState() const {
        return m_state.state;
    }

    void Generator::resetState() {
        m_state.state = State::Undefined; // movement state

        m_state.currentDirection = Direction::Neutral; // current direction
        m_state.targetDirection  = Direction::Neutral; // target direction

        m_state.doDirectionChange = false; // request direction change

        // Kinematic state
        m_state.currentVelocity = 0.0f; // steps/s
        m_state.targetVelocity  = 0.0f; // steps/s
        m_state.acceleration    = 0.0f; // steps/s^2
        m_state.deceleration    = 0.0f; // steps/s^2

        // Distance mode state
        m_state.stepsTotal = 0; // total steps requested
        m_state.stepsDone  = 0; // steps already executed
        m_state.stepsAcc   = 0; // steps in acceleration phase
        m_state.stepsConst = 0; // steps in constant velocity phase
        m_state.stepsDec   = 0; // steps in deceleration phase
    }

    void IRAM_ATTR Generator::timerCallback(void* arg) {
        auto* self = static_cast<Generator*>(arg);

        if (!compare_enums(self->m_state.state, State::Running)) {
            return;
        }

        // Produce one step
        self->m_driver.doStep();

        // Schedule next step if still running
        if (self->advanceStateAfterStep(self->m_state)) {
            uint64_t period_us = self->computeStepPeriodUs(self->m_state.currentVelocity);
            self->scheduleNextStep(period_us);
        }
    }

    bool IRAM_ATTR Generator::gptimerOnAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
        auto* self = static_cast<Generator*>(user_ctx);
        bool ret = false;

        // Produce one step
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        self->m_driver.doStepFromISR(&xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            ret = true;
        }

        // Check stepper task queue
        // StepperTask task;
        // xHigherPriorityTaskWoken = pdFALSE;
        // if(xQueueReceiveFromISR(self->m_stepQueueHandle, (void*) &task, &xHigherPriorityTaskWoken)) {
        //     gptimer_alarm_config_t alarm_config;
        //     alarm_config.alarm_count = 20;
        //     alarm_config.reload_count = 0;
        //     alarm_config.flags.auto_reload_on_alarm = true;
        //     ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

        //     // gptimer_alarm_config_t alarm_config;
        //     // alarm_config.alarm_count = edata->alarm_value + task.period_us;
        //     // alarm_config.flags.auto_reload_on_alarm = false;
        //     // ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config)); 
        // }
        // if (xHigherPriorityTaskWoken == pdTRUE) {
        //     ret = true;
        // }



        // if () {
        //     gptimer_alarm_config_t alarm_config;
        //     alarm_config.alarm_count = edata->alarm_value + task.period_us;
        //     ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
        // }
        // else {
        //     // Stop timer and reset
        //     ESP_ERROR_CHECK(gptimer_stop(timer));
        //     ESP_ERROR_CHECK(gptimer_set_raw_count(timer, 0));
        // }

    //     // // Update the alarm value
    //     // gptimer_set_alarm_action(timer, &alarm_config);
    //     ESP_ERROR_CHECK(gptimer_set_raw_count(timer, period_us));

/*
        /////////////////////////////////////////////////////////////

        float dt = 1.0f / (self->m_state.currentVelocity > self->k_minVelocity) ? self->m_state.currentVelocity : self->k_minVelocity;
        if (self->m_state.stepsTotal == 0) {
            // Velocity mode
            if (self->m_state.targetVelocity <= 0.0f) {
                // Decelerate to stop
                if (self->m_state.currentVelocity > 0.0f) {
                    self->m_state.state = State::Decelerating;
                    self->m_state.currentVelocity = std::max(0.0f, self->m_state.currentVelocity - self->m_state.deceleration * dt);
                }
                if (self->m_state.currentVelocity <= 0.0f) {
                    self->m_state.state = State::Stopped;
                    ESP_ERROR_CHECK(gptimer_stop(timer));
                    return ret;
                }
            } else {
                if (self->m_state.currentDirection != self->m_state.targetDirection) {
                    // Need to change direction: decelerate to stand-still first
                    if (self->m_state.currentVelocity > 0.0f) {
                        self->m_state.state = State::Decelerating;
                        self->m_state.currentVelocity = std::max(0.0f, self->m_state.currentVelocity - self->m_state.deceleration * dt);
                    }
                    if (self->m_state.currentVelocity <= 0.0f) {
                        // Reached stand-still, change direction
                        self->m_state.state = State::Running;

                        xHigherPriorityTaskWoken = pdFALSE;
                        self->m_driver.setDirectionFromISR(self->m_state.targetDirection, &xHigherPriorityTaskWoken);
                        if (xHigherPriorityTaskWoken == pdTRUE) {
                            //portYIELD_FROM_ISR();
                            ret = true;
                        }

                        self->m_state.currentDirection = self->m_state.targetDirection;
                        self->m_state.currentVelocity = self->k_minVelocity;
                    }
                }
                else {
                    // Move towards target velocity
                    if (self->m_state.currentVelocity < self->m_state.targetVelocity) {
                        self->m_state.state = State::Accelerating;
                        //state.currentVelocity = std::min(state.targetVelocity, state.currentVelocity + state.acceleration * dt);
                        float newVelocity = self->m_state.currentVelocity + self->m_state.acceleration * dt;
                        self->m_state.currentVelocity = (self->m_state.targetVelocity < newVelocity) ? self->m_state.targetVelocity : newVelocity;
                    }
                    else if (self->m_state.currentVelocity > self->m_state.targetVelocity) {
                        self->m_state.state = State::Decelerating;
                        float newVelocity = self->m_state.currentVelocity - self->m_state.acceleration * dt;
                        //state.currentVelocity = std::max(state.targetVelocity, state.currentVelocity - state.deceleration * dt);
                        self->m_state.currentVelocity = (self->m_state.targetVelocity > newVelocity) ? self->m_state.targetVelocity : newVelocity;
                    }
                    else {
                        self->m_state.state = State::Running;
                    }
                }
            }
        }
        else {
            // Distance mode
            self->m_state.stepsDone = self->m_state.stepsDone + 1;

            // Determine phase by stepsDone
            if (self->m_state.stepsDone <= self->m_state.stepsAcc) {
                // Accelerating: v^2 = v0^2 + 2 a s; step-by-step approx
                self->m_state.state = State::Accelerating;
                //state.currentVelocity = std::min(state.targetVelocity, state.currentVelocity + state.acceleration * dt);
                float newVelocity = self->m_state.currentVelocity + self->m_state.acceleration * dt;
                self->m_state.currentVelocity = (self->m_state.targetVelocity < newVelocity) ? self->m_state.targetVelocity : newVelocity;
            } else if (self->m_state.stepsDone <= (self->m_state.stepsAcc + self->m_state.stepsConst)) {
                // Constant
                self->m_state.state = State::Running;
                self->m_state.currentVelocity = self->m_state.targetVelocity;
            } else if (self->m_state.stepsDone <= (self->m_state.stepsAcc + self->m_state.stepsConst + self->m_state.stepsDec)) {
                // Decelerating
                self->m_state.state = State::Decelerating;
                self->m_state.currentVelocity = std::max(0.0f, self->m_state.currentVelocity - self->m_state.deceleration * dt);
            }

            if (self->m_state.stepsDone >= self->m_state.stepsTotal) {
                self->m_state.state = State::Stopped;
                ESP_ERROR_CHECK(gptimer_stop(timer));
                return ret;
            }
        }

        if (self->m_state.currentVelocity <= 0.0f) {
            ESP_ERROR_CHECK(gptimer_stop(timer));
            return ret;
        }

        // period = 1e6 / v (microseconds per step)
        float p = 1000000.0f / self->m_state.currentVelocity;

        // Clamp to a sane range
        //p = std::max(static_cast<float>(m_driver.getMinPulsePeriodUs()), p);
        //p = std::min(static_cast<float>(m_driver.getMaxPulsePeriodUs()), p);

        
    //     //self->scheduleNextStep(period_us);
        gptimer_alarm_config_t alarm_config;
        alarm_config.alarm_count = edata->alarm_value + static_cast<uint64_t>(p);

    //     // // Update the alarm value
    //     // gptimer_set_alarm_action(timer, &alarm_config);
    //     ESP_ERROR_CHECK(gptimer_set_raw_count(timer, period_us));

        ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
    //     ESP_ERROR_CHECK(gptimer_start(timer));
*/
        return ret;
    }

    void IRAM_ATTR Generator::generatorTask(void* args) {
        auto* self = static_cast<Generator*>(args);

        for (;;) {
            TickType_t waitTicks = 0;
            if (!compare_enums(self->m_state.state, State::Running)) {
                // If not running wait for new task
                ESP_LOGI(log_tag, "Waiting for new generator task...");
                waitTicks = portMAX_DELAY;
            }

            // Check queue for new task
            GeneratorTask generatorTask;
            if (xQueueReceive(self->m_taskQueueHandle, &generatorTask, waitTicks)) {
                if (self->initializeStateBeforeStep(generatorTask, self->m_state)) {

                    // Reset stepper queue
                    xQueueReset(self->m_stepQueueHandle);

                    // Set driver direction
                    self->m_driver.setDirection(self->m_state.targetDirection);
                    self->m_state.currentDirection = self->m_state.targetDirection;

                    // Calc first step period
                    uint64_t period_us = self->computeStepPeriodUs(self->m_state.currentVelocity);
                    if (period_us > 0) {
                        self->m_state.state = State::Running;

                        // Start timer
                        self->rearmTimer(period_us);

                        // Execute first step
                        self->m_driver.doStep();
                    }
                    else {
                        self->m_state.state = State::Stopped;
                    }

                }
            }

            // Propagate internal states for next period
            if (self->advanceStateAfterStep(self->m_state)) {
                // Check if direction change is pending
                if (self->m_state.doDirectionChange) {
                    self->m_state.currentDirection = self->m_driver.changeDirection();
                    self->m_state.doDirectionChange = false;
                }

                // Compute next step period
                StepperTask stepTask {
                    .period_us = self->computeStepPeriodUs(self->m_state.currentVelocity),
                };

                // Send new stepper task to queue and block, until queue has been emptied
                xQueueSend(self->m_stepQueueHandle, (void*) &stepTask, portMAX_DELAY);
            }
        }
    }

    void Generator::scheduleNextStep(uint64_t period_us) {
        if (period_us == 0) {
            stopTimer();
            m_state.state = State::Stopped;
            return;
        }

        rearmTimer(period_us);
    }

    uint64_t IRAM_ATTR Generator::computeStepPeriodUs(float velocity) const {
        // Compute period from velocity
        if (velocity <= 0.0f) {
            return 0; // default 0 us when stopped
        }
        // period = 1e6 / v (microseconds per step)
        float p = 1000000.0f / velocity;

        // Clamp to a sane range
        p = std::max(static_cast<float>(m_driver.getMinPulsePeriodUs()), p);
        p = std::min(static_cast<float>(m_driver.getMaxPulsePeriodUs()), p);

        return static_cast<uint64_t>(p);
    }

    bool IRAM_ATTR Generator::advanceStateAfterStep(GeneratorState& state, float minVelocity) {
        
        float dt = 1.0f / (state.currentVelocity > minVelocity) ? state.currentVelocity : minVelocity;
        if (state.stepsTotal == 0) {
            // Velocity mode
            if (state.targetVelocity <= 0.0f) {
                // Decelerate to stop
                if (state.currentVelocity > 0.0f) {
                    state.state = State::Decelerating;
                    state.currentVelocity = std::max(0.0f, state.currentVelocity - state.deceleration * dt);
                }
                if (state.currentVelocity <= 0.0f) {
                    state.state = State::Stopped;
                    return false;
                }
            } else {
                if (state.currentDirection != state.targetDirection) {
                    // Need to change direction: decelerate to stand-still first
                    if (state.currentVelocity > 0.0f) {
                        state.state = State::Decelerating;
                        state.currentVelocity = std::max(0.0f, state.currentVelocity - state.deceleration * dt);
                    }
                    if (state.currentVelocity <= 0.0f) {
                        // Reached stand-still, change direction
                        state.state = State::Running;
                        state.doDirectionChange = true;
                        state.currentVelocity = minVelocity;
                    }
                }
                else {
                    // Move towards target velocity
                    if (state.currentVelocity < state.targetVelocity) {
                        state.state = State::Accelerating;
                        //state.currentVelocity = std::min(state.targetVelocity, state.currentVelocity + state.acceleration * dt);
                        float newVelocity = state.currentVelocity + state.acceleration * dt;
                        state.currentVelocity = (state.targetVelocity < newVelocity) ? state.targetVelocity : newVelocity;
                    }
                    else if (state.currentVelocity > state.targetVelocity) {
                        state.state = State::Decelerating;
                        float newVelocity = state.currentVelocity - state.acceleration * dt;
                        //state.currentVelocity = std::max(state.targetVelocity, state.currentVelocity - state.deceleration * dt);
                        state.currentVelocity = (state.targetVelocity > newVelocity) ? state.targetVelocity : newVelocity;
                    }
                    else {
                        state.state = State::Running;
                    }
                }
            }
        }
        else {
            // Steps mode
            state.stepsDone++;

            // Determine phase by stepsDone
            if (state.stepsDone <= state.stepsAcc) {
                // Accelerating: v^2 = v0^2 + 2 a s; step-by-step approx
                state.state = State::Accelerating;
                //state.currentVelocity = std::min(state.targetVelocity, state.currentVelocity + state.acceleration * dt);
                float newVelocity = state.currentVelocity + state.acceleration * dt;
                state.currentVelocity = (state.targetVelocity < newVelocity) ? state.targetVelocity : newVelocity;
            } else if (state.stepsDone <= (state.stepsAcc + state.stepsConst)) {
                // Constant
                state.state = State::Running;
                state.currentVelocity = state.targetVelocity;
            } else if (state.stepsDone <= (state.stepsAcc + state.stepsConst + state.stepsDec)) {
                // Decelerating
                state.state = State::Decelerating;
                state.currentVelocity = std::max(0.0f, state.currentVelocity - state.deceleration * dt);
            }

            if (state.stepsDone >= state.stepsTotal) {
                state.state = State::Stopped;
                return false;
            }
        }
        return true;
    }

} // namespace Stepper

    // /*
    // Calculate the steps needed to reach the desired steps/s velocity:

    // steps_needed = (v1^2 - v0^2) / 2 * a

    // The time to finish the movement is calculated as:

    // t = (v1 - v0) / a

    // The time needs to be divided by the needed steps



    // We now know hom many steps we need to reach final velocity.
    // We now have to calculate the time between steps.
    // For a given constant speed v in steps/s the time between two steps is 1/v
    // For an accelerated movement, the time difference between steps needs to be adapted

    // v(t) = a*t
    // s(t) = v0*t + 0.5*a*t^2
    // x(t) = x0 + v0*t + 0.5*a*t^2

    // t1 = t0 * (1 - a * t0^2)
    // t1 = t0 - a * t0^3
    // */

    // /*
    // void ESP_FlexyStepper::setSpeedInStepsPerSecond(float speedInStepsPerSecond)
    // {
    //   desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
    //   desiredPeriod_InUSPerStep = 1000000.0 / desiredSpeed_InStepsPerSecond;
    // }

    // void ESP_FlexyStepper::setAccelerationInStepsPerSecondPerSecond(
    //     float accelerationInStepsPerSecondPerSecond)
    // {
    //   acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
    //   acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;

    //   periodOfSlowestStep_InUS =
    //       1000000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);
    //   minimumPeriodForAStoppedMotion = periodOfSlowestStep_InUS / 2.8;
    // }

    //   //
    //   // determine the number of steps needed to go from the current speed down to a
    //   // velocity of 0, Steps = Velocity^2 / (2 * Deceleration)
    //   //
    //   currentStepPeriodSquared = currentStepPeriod_InUS * currentStepPeriod_InUS;
    //   decelerationDistance_InSteps = (long)round(
    //       5E11 / (deceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared));

    // // NextStepPeriod = CurrentStepPeriod(1 + acceleration * CurrentStepPeriod^2)
    // // NextStepPeriod = CurrentStepPeriod(1 - deceleration * CurrentStepPeriod^2)
    // */
    // // uint64_t Generator::calcStepsNeeded(float deltaVelocity, float acceleration)
    // // {
    // //     /*
    // //     Calculate the steps (s) needed to reach the desired velocity (v) with the given acceleration (a):

    // //     s = 0.5 * a * t² = 0.5 * v² / a
    // //     */
    // //     if (acceleration == 0)
    // //         return 0;

    // //     return 0.5 * deltaVelocity * deltaVelocity / acceleration;
    // // }

    // // uint64_t Generator::calcStepsNeeded(float currentVelocity, float targetVelocity, float acceleration)
    // // {
    // //     return calcStepsNeeded(targetVelocity - currentVelocity, acceleration);
    // // }

    // // uint64_t Generator::calcStepPeriodInTicks(uint64_t steps, float acceleration, uint64_t ticksPerSecond)
    // // {
    // //     /*
    // //     The time (t) to finish the entire movement is calculated as:

    // //     t = v / a

    // //     The velocity at a given step can be calculated as:

    // //     v = sqrt(2*s*a)

    // //     For a given v in steps/s the time between two steps is:

    // //     t = 1 / v = 1 / sqrt(2*s*a)

    // //     s = 0.5 * a * t²
    // //     2*a*s = (a*t)² = v² = (ticks_per_s/ticks)²
    // //     ticks = ticks_per_s / sqrt(2*a*s)
    // //     ticks = ticks_per_s / sqrt(2) / sqrt(a*s)
    // //     */
    // //     if (acceleration == 0.0)
    // //         return 0;

    // //     if (steps == 0)
    // //         return MAX_STEP_PERIOD_IN_TICKS;

    // //     return ticksPerSecond / sqrt(2 * steps * acceleration);
    // // }

    // // uint64_t Generator::calcStepPeriodInTicks(float velocity, uint64_t ticksPerSecond)
    // // {
    // //     /*
    // //     For a given v in steps/s the time between two steps is:

    // //     t = 1 / v
    // //     */

    // //     if (velocity == 0.0)
    // //         return MAX_STEP_PERIOD_IN_TICKS;

    // //     return ticksPerSecond / velocity;
    // // }