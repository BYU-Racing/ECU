#include "speed.h"

float speed_sensor_get_voltage(uint8_t pin){
    float out = analogRead(pin);
    out = (out/TEENSY_CONSTANT)*TS_VOLTAGE_MAX_CONSTANT/DIVIDER_CONSTANT;
    return(out);
}

float speed_sensor_get_speed(uint8_t pin){
    float out = speed_sensor_get_voltage(pin);
    out = 200 * out - 100;
    return(out);
}