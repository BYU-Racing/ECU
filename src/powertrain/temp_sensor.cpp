#include "temp_sensor.h"
#include <math.h>

float temp_sensor_get_voltage(uint8_t pin){
    float out = analogRead(pin);
    out = (out/TEENSY_CONSTANT)*TS_VOLTAGE_MAX_CONSTANT;
    return(out);
}

float temp_sensor_get_temp(uint8_t pin,uint8_t mode){
    float out = temp_sensor_get_voltage(pin);
    switch (mode)
    {
    case 0:
        out = TS_LOG_CO*log(out)+TS_OFFSET;
        break;
    case 1:
        out = (TS_A_CO*out*out*out)+(TS_B_CO*out*out)+(TS_C_CO*out)+(TS_D_CO);
        break;
    default:
        out = -1;
        break;
    }
    return(out);
}