#pragma once
#include <Arduino.h>

#define PS_VOLTAGE_MAX_CONSTANT (3.3*2)
#define TEENSY_CONSTANT 1023

float pressure_sensor_get_voltage(uint8_t pin);

float pressure_sensor_get_pressure(uint8_t pin);