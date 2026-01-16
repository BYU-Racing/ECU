#pragma once
#include <Arduino.h>

#define TEENSY_CONSTANT 1023
#define TS_VOLTAGE_MAX_CONSTANT 3.3
#define DIVIDER_CONSTANT .6

float speed_sensor_get_voltage(uint8_t pin);

float speed_sensor_get_speed(uint8_t pin);