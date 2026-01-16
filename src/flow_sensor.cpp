#include "flow_sensor.h"

static uint8_t lastHighLowRead = 0;
static uint32_t flowRateTimer = 0;
static uint16_t numRises = 0;
static float readFrequency;

void flowISR(){
    uint8_t highLowRead = digitalRead(FLOW_SENSOR_PIN);
    //check for no change and then do nothing
    if(highLowRead == lastHighLowRead){
        ++flowRateTimer;
        return;
    }
    lastHighLowRead = highLowRead;
    if(highLowRead == HIGH){
        /*counting the number of time we are riseing
        or low to high pulses on the sensor
        Note that the sensor is NOT PWM it is changeing frequncy
        in stead*/
        ++numRises;
    }
    if(numRises < FLOW_SENSOR_NUM_DATA){
        //do nothing if we dont have enough data points
        return;
    }
    
    if(flowRateTimer > 0){//devide my 0 protection
        readFrequency = SAMPLE_FREQUENCY_HZ / flowRateTimer;
    }
    flowRateTimer = 0;
    numRises = 0;
}

float flowRate(){
    return(readFrequency / (FLOW_SENSOR_CALIBRATION * LITER_TIME_MULTIPLYER));
}