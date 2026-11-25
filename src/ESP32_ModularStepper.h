#ifndef ESP32_MODULAR_STEPPER_H
#define ESP32_MODULAR_STEPPER_H

#include "Generic/Helper.hpp"
#include "Generic/Log.hpp"
#include "Generic/Pin.hpp"
#include "Generic/Driver.hpp"
#include "Generic/Generator.hpp"
#include "Generic/Motor.hpp"
#include "Generic/Endstop.hpp"
#include "Generic/Axis.hpp"
#include "Generic/Axis_Linear.hpp"
#include "Generic/Axis_Rotary.hpp"
#include "Generic/Homing.hpp"
#include "Generic/StopController.hpp"
#include "Generic/Config.hpp"

#if defined(ESP_PLATFORM) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#include "ESP32/Pin.hpp"
#include "ESP32/Driver.hpp"
#include "ESP32/Driver_MCPWM.hpp"
#include "ESP32/Generator.hpp"
#include "ESP32/Endstop.hpp"
#include "ESP32/Homing.hpp"
#include "ESP32/Event.hpp"
#include "ESP32/Assembly.hpp"
#endif

#endif // ESP32_MODULAR_STEPPER_H
