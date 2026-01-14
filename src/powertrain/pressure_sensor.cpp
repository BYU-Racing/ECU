#include "pressure_sensor.h"

float pressure_sensor_get_voltage(uint8_t pin){
    float out = analogRead(pin);
    out = (out/TEENSY_CONSTANT)*PS_VOLTAGE_MAX_CONSTANT;
    return(out);
}

float pressure_sensor_get_pressure(uint8_t pin){
    float out = pressure_sensor_get_voltage(pin);
    return(out);
}