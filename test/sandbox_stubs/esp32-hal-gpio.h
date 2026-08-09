#ifndef ESP32_HAL_GPIO_H
#define ESP32_HAL_GPIO_H

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0

inline void digitalWrite(int pin, int val) { (void)pin; (void)val; }
inline int digitalRead(int pin) { (void)pin; return 0; }
inline void pinMode(int pin, int mode) { (void)pin; (void)mode; }

#endif // ESP32_HAL_GPIO_H
