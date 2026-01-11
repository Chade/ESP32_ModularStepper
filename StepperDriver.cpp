#include "StepperDriver.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/pcnt_types.h"
#include <driver/gpio.h>
#include <limits>

namespace Stepper
{
    static const char* LOG_TAG = "Driver";

    Driver::Driver(uint8_t enablePin, uint8_t stepPin, uint8_t directionPin, bool inverseDirection) : m_pinEnable(enablePin), m_pinStep(stepPin), m_pinDirection(directionPin), m_inverseDirection(inverseDirection)
    {
        // Setup gpio pins
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)m_pinEnable, GPIO_MODE_INPUT_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)m_pinDirection, GPIO_MODE_INPUT_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)m_pinStep, GPIO_MODE_INPUT_OUTPUT));

        // Create counter unit
        pcnt_unit_config_t pcntUnitConfig = {
            .high_limit = std::numeric_limits<uint16_t>::max(),
            .low_limit = std::numeric_limits<uint16_t>::min(),
            .flags.accum_count = 1,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&pcntUnitConfig, &m_pcntUnitHandle));

        // Create counter channel
        pcnt_chan_config_t pcntChannelConfig = {
            .edge_gpio_num = (gpio_num_t)m_stepPin,
            .level_gpio_num = (gpio_num_t)m_pinDirection,
        };
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

    void Driver::stepTask(void *args)
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
                esp_rom_delay_us(self->m_stepPulseWidthLow_us);
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
                uxTaskPriorityGet(NULL) + 1, /* Task priority */
                &m_taskHandle,               /* Task handle */
                1                            /* CPU core to use */
            );
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Already started!");
        }
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
            ESP_LOGE(LOG_TAG, "Already stopped!");
        }
    }

    void Driver::setTimings(float stepPulseWidthHigh_us, float stepPulseWidthLow_us, float directionDelay_ns, uint32_t maxPulsePeriodUs)
    {
        m_stepPulseWidthHigh_us = stepPulseWidthHigh_us;
        m_stepPulseWidthLow_us = stepPulseWidthLow_us;
        m_directionDelay_ns = directionDelay_ns;
        m_maxPulsePeriodUs = maxPulsePeriodUs;
    }

    uint32_t Driver::getMinPulsePeriodUs()
    {
        return m_stepPulseWidthHigh_us + m_stepPulseWidthLow_us;
    }

    uint32_t Driver::getMaxPulsePeriodUs()
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
            ESP_LOGE(LOG_TAG, "Step task not started!");
        }
    }

    void Driver::doStepFromISR(BaseType_t *pxHigherPriorityTaskWoken)
    {
        if (m_taskHandle != nullptr)
        {
            BaseType_t xHigherPriorityTaskWokenLocal = pdFALSE;
            BaseType_t *pFlag = pxHigherPriorityTaskWoken ? pxHigherPriorityTaskWoken : &xHigherPriorityTaskWokenLocal;
            xTaskNotify(m_taskHandle, ulDoStepBitmask, eNotifyAction::eSetBits, pFlag);
        }
    }

    void Driver::setDirection(Direction direction)
    {
        uint32_t directionBit;
        if (direction == Direction::CLOCKWISE)
        {
            directionBit = m_inverseDirection << 2;
        }
        else if (direction == Direction::COUTERCLOCKWISE)
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
            xTaskNotify(m_taskHandle, ulDoDirectionChangeBitmask | directionBit, eNotifyAction::eSetBits);
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Step task not started!");
        }
    }

    void Driver::setDirectionFromISR(Direction direction, BaseType_t *pxHigherPriorityTaskWoken)
    {
        uint32_t directionBit;
        if (direction == Direction::CLOCKWISE)
        {
            directionBit = m_inverseDirection << 2;
        }
        else if (direction == Direction::COUTERCLOCKWISE)
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
            xTaskNotify(m_taskHandle, ulDoDirectionChangeBitmask | directionBit, eNotifyAction::eSetBits, pFlag);
        }
    }

    Direction Driver::getDirection()
    {
        if (!isEnabled())
        {
            return Direction::NEUTRAL;
        }

        if (gpio_get_level((gpio_num_t)m_pinDirection) == m_inverseDirection)
            return Direction::CLOCKWISE;
        else
            return Direction::COUNTERCLOCKWISE;
    }

    Direction Driver::changeDirection()
    {
        if (!isEnabled())
        {
            return Direction::NEUTRAL;
        }

        uint8_t direction = !gpio_get_level((gpio_num_t)m_pinDirection);
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)m_pinDirection, direction));

        if (direction == m_inverseDirection)
            return Direction::CLOCKWISE;
        else
            return Direction::COUNTERCLOCKWISE;
    }

    void Driver::enable()
    {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)m_pinEnable, 0));
    }

    void Driver::disable()
    {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)m_pinEnable, 1));
    }

    bool Driver::isEnabled()
    {
        return gpio_get_level((gpio_num_t)m_pinEnable);
    }

    int32_t Driver::getCount()
    {
        int32_t value = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(m_pcntUnitHandle, &value));
        return value;
    }

    void Driver::resetCount()
    {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(m_pcntUnitHandle));
    }
}