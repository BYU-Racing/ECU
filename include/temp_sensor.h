#pragma once
#include <Arduino.h>

#define TEENSY_CONSTANT 1023
#define TS_VOLTAGE_MAX_CONSTANT (3.3 * 2)
//443 Ohms is probobly the resistor to put in series with the thermistor
#define TS_LOG_CO -51.3346
#define TS_OFFSET 81.2540

#define TS_A_CO -4.4382
#define TS_B_CO 38.0821
#define TS_C_CO -121.2466
#define TS_D_CO 174.6725

float temp_sensor_get_voltage(uint8_t pin);

float temp_sensor_get_temp(uint8_t pin,uint8_t mode);