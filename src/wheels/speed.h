#pragma once
#include <Arduino.h>

#define SAMPLE_FREQUENCY_HZ 1000
#define SPEED_CALIBRATION_CONSTANT 1
#define SPEED_SENSOR_PERIOD ((1.0f/SAMPLE_FREQUENCY_HZ)*1000000.0f)
#define SAMPLE_COUNT 2 //number of rises that it uses to find average

void setSpeedPin(uint8_t inp);

void speedISR();

void calculateSpeed();

float getSpeed();