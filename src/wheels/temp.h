#pragma once
#include <Arduino.h>

#define TEENSY_CONSTANT 1023
#define TS_VOLTAGE_MAX_CONSTANT 3.3
#define DIVIDER_CONSTANT .6

float temp_sensor_get_voltage(uint8_t pin);

float temp_sensor_get_temp(uint8_t pin);