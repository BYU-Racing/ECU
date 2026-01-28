#pragma once
#include <Arduino.h>

#define TEENSY_CONSTANT 1023
#define TS_VOLTAGE_MAX_CONSTANT 3.3
#define DIVIDER_CONSTANT .5
#define SENSOR_VOLTAGE_RANGE 5

float brake_pot_get_voltage(uint8_t pin);

float brake_pot_get_depression(uint8_t pin);