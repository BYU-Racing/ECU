#include "brake.h"

float brake_pot_get_voltage(uint8_t pin){
    float out = analogRead(pin);
    out = (out/TEENSY_CONSTANT)*TS_VOLTAGE_MAX_CONSTANT/DIVIDER_CONSTANT;
    return(out);
}

float brake_pot_get_depression(uint8_t pin){
    float voltage = brake_pot_get_voltage(pin);
    float percent_depression = voltage / SENSOR_VOLTAGE_RANGE;

    return percent_depression;
}