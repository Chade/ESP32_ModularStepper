#include "StepperDriver.h"
#include "StepperLog.h"
#include <driver/gpio.h>
#include <limits>

namespace Stepper
{
    Driver::Driver(uint8_t enablePin, uint8_t stepPin, uint8_t directionPin, bool inverseDirection) : m_pinEnable(enablePin), m_pinStep(stepPin), m_pinDirection(directionPin), m_inverseDirection(inverseDirection)
    {
        esp_log_level_set(log_tag, ESP_LOG_INFO);

        // Setup gpio pins
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)m_pinEnable, GPIO_MODE_INPUT_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)m_pinStep, GPIO_MODE_INPUT_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)m_pinDirection, GPIO_MODE_INPUT_OUTPUT));

        // Create counter unit
        pcnt_unit_config_t pcntUnitConfig;
        pcntUnitConfig.low_limit = -100;
        pcntUnitConfig.high_limit = 100;
        pcntUnitConfig.intr_priority = 0;
        //pcntUnitConfig.flags.accum_count = 1;

        ESP_ERROR_CHECK(pcnt_new_unit(&pcntUnitConfig, &m_pcntUnitHandle));

        // Create counter channel
        pcnt_chan_config_t pcntChannelConfig;
        pcntChannelConfig.edge_gpio_num = (gpio_num_t)m_pinStep;
        pcntChannelConfig.level_gpio_num = (gpio_num_t)m_pinDirection;

        ESP_ERROR_CHECK(pcnt_new_channel(m_pcntUnitHandle, &pcntChannelConfig, &m_pctChannelHandle));

        // Define counting behaviour
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(m_pctChannelHandle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(m_pctChannelHandle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    }

    Driver::~Driver()
    {
        stop();
        pcnt_del_channel(m_pctChannelHandle);
        pcnt_del_unit(m_pcntUnitHandle);
    }

    void IRAM_ATTR Driver::stepTask(void *args)
    {
        Driver *self = static_cast<Driver *>(args);

        uint32_t ulNotifiedValue;

        for (;;)
        {
            if (xTaskNotifyWait(0, ulDoStepBitmask | ulDoDirectionChangeBitmask | ulDirectionBitmask, &ulNotifiedValue, portMAX_DELAY) != pdTRUE)
            {
                continue; // Timeout or error
            }

            if (ulNotifiedValue & ulDoDirectionChangeBitmask)
            {
                gpio_set_level((gpio_num_t)self->m_pinDirection, (ulNotifiedValue & ulDirectionBitmask));

                uint32_t delay_us = static_cast<uint32_t>((self->m_directionDelay_ns + 999) / 1000); // ceil(ns/1000)

                if (delay_us > 0)
                {
                    esp_rom_delay_us(delay_us);
                }
            }

            if (ulNotifiedValue & ulDoStepBitmask)
            {
                // Generate the step pulse
                gpio_set_level((gpio_num_t)self->m_pinStep, 1);
                esp_rom_delay_us(self->m_stepPulseWidthHigh_us);
                gpio_set_level((gpio_num_t)self->m_pinStep, 0);
                //esp_rom_delay_us(self->m_stepPulseWidthLow_us);
            }
        }
    }

    void Driver::start()
    {
        ESP_ERROR_CHECK(pcnt_unit_enable(m_pcntUnitHandle));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(m_pcntUnitHandle));
        ESP_ERROR_CHECK(pcnt_unit_start(m_pcntUnitHandle));

        // Create step task
        if (m_taskHandle == nullptr)
        {
            xTaskCreatePinnedToCore(
                Driver::stepTask,            /* Task function */
                "StepTask",                  /* Task name (max. 16 characters by default) */
                2048,                        /* Stack size in bytes */
                this,                        /* Parameter passed as input of the task */
                12,                          /* Task priority */
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

    void Driver::stop()
    {
        ESP_ERROR_CHECK(pcnt_unit_stop(m_pcntUnitHandle));
        ESP_ERROR_CHECK(pcnt_unit_disable(m_pcntUnitHandle));

        // Delete step task
        if (m_taskHandle != nullptr)
        {
            vTaskDelete(m_taskHandle);
            m_taskHandle = nullptr;
        }
        else
        {
            ESP_LOGE(log_tag, "Task already stopped!");
        }

        ESP_LOGI(log_tag, "Stopped");
    }

    void Driver::setTimings(uint32_t stepPulseWidthHigh_us, uint32_t stepPulseWidthLow_us, uint32_t directionDelay_ns, uint32_t maxPulsePeriodUs)
    {
        m_stepPulseWidthHigh_us = stepPulseWidthHigh_us;
        m_stepPulseWidthLow_us = stepPulseWidthLow_us;
        m_directionDelay_ns = directionDelay_ns;
        m_maxPulsePeriodUs = maxPulsePeriodUs;
        ESP_LOGI(log_tag, "Timings set");
    }

    void Driver::registerCallback(DriverCallback callback)
    {
        m_callback = callback;
    }

    uint32_t Driver::getMinPulsePeriodUs() const
    {
        return m_stepPulseWidthHigh_us + m_stepPulseWidthLow_us;
    }

    uint32_t Driver::getMaxPulsePeriodUs() const
    {
        return m_maxPulsePeriodUs;
    }

    void Driver::doStep()
    {
        if (m_taskHandle != nullptr)
        {
            // Just notify the step task
            xTaskNotify(m_taskHandle, ulDoStepBitmask, eNotifyAction::eSetBits);
        }
        else
        {
            ESP_LOGE(log_tag, "Step task not started!");
        }
    }

    void IRAM_ATTR Driver::doStepFromISR(BaseType_t *pxHigherPriorityTaskWoken)
    {
        if (m_taskHandle != nullptr)
        {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t *pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            xTaskNotifyFromISR(m_taskHandle, ulDoStepBitmask, eNotifyAction::eSetBits, pFlag);
        }
    }

    void Driver::setDirection(Direction direction)
    {
        uint32_t directionBit;
        if (direction == Direction::Clockwise)
        {
            ESP_LOGI(log_tag, "Clockwise");
            directionBit = m_inverseDirection << 2;
        }
        else if (direction == Direction::Counterclockwise)
        {
            ESP_LOGI(log_tag, "Counterclockwise");
            directionBit = !m_inverseDirection << 2;
        }
        else
        {
            ESP_LOGI(log_tag, "Neutral");
            disable();
            return;
        }

        if (m_taskHandle != nullptr)
        {
            xTaskNotify(m_taskHandle, ulDoDirectionChangeBitmask | directionBit, eNotifyAction::eSetBits);
        }
        else
        {
            ESP_LOGE(log_tag, "Step task not started!");
        }
    }

    void IRAM_ATTR Driver::setDirectionFromISR(Direction direction, BaseType_t *pxHigherPriorityTaskWoken)
    {
        uint32_t directionBit;
        if (direction == Direction::Clockwise)
        {
            directionBit = m_inverseDirection << 2;
        }
        else if (direction == Direction::Counterclockwise)
        {
            directionBit = !m_inverseDirection << 2;
        }
        else
        {
            disable();
            return;
        }

        if (m_taskHandle != nullptr)
        {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t *pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            xTaskNotifyFromISR(m_taskHandle, ulDoDirectionChangeBitmask | directionBit, eNotifyAction::eSetBits, pFlag);
        }
    }

    Direction Driver::getDirection() const
    {
        if (!isEnabled())
        {
            return Direction::Neutral;
        }

        if (gpio_get_level((gpio_num_t)m_pinDirection) == m_inverseDirection)
            return Direction::Clockwise;
        else
            return Direction::Counterclockwise;
    }

    Direction Driver::changeDirection()
    {
        if (!isEnabled())
        {
            return Direction::Neutral;
        }

        uint8_t direction = !gpio_get_level((gpio_num_t)m_pinDirection);
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)m_pinDirection, direction));

        if (direction == m_inverseDirection)
            return Direction::Clockwise;
        else
            return Direction::Counterclockwise;
    }

    void Driver::enable()
    {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)m_pinEnable, 1));
        ESP_LOGI(log_tag, "Enabled");
    }

    void Driver::disable()
    {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)m_pinEnable, 0));
        ESP_LOGI(log_tag, "Disabled");
    }

    bool Driver::isEnabled() const
    {
        return gpio_get_level((gpio_num_t)m_pinEnable);
    }

    int32_t Driver::getCount() const
    {
        int value = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(m_pcntUnitHandle, &value));
        return value;
    }

    void Driver::resetCount()
    {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(m_pcntUnitHandle));
        ESP_LOGI(log_tag, "Counter reset");
    }
}