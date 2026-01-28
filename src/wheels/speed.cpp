#include "speed.h"

static uint8_t lastHighLowRead = 0;
static uint32_t flowRateTimer = 0;
static uint16_t numRises = 0;
static uint8_t speedPin = 0;
static float readFrequency = 0;

void setSpeedPin(uint8_t inp) {
    speedPin = inp;
}

void speedISR(){
    checkSpeed = true;
}

void calculateSpeed() {
    if (!checkSpeed) return;
    //Serial.println("CHECKING SPEED");
    uint8_t highLowRead = (analogRead(speedPin) > 300);
    //if (highLowRead) Serial.println("SENSOR HIGH");
    //if (highLowRead) { Serial.println("DEBUG: SENSOR PIN IS HIGH"); }
    //check for no change and then do nothing
    if(highLowRead == lastHighLowRead){
        ++flowRateTimer;
        return;
    } else {
        lastHighLowRead = highLowRead;
        if(highLowRead) {
            /*counting the number of time we are riseing
            or low to high pulses on the sensor
            Note that the sensor is NOT PWM it is changeing frequncy
            in stead*/
            ++numRises;
        }

        if(numRises < SAMPLE_COUNT){
            //do nothing if we dont have enough data points
            return;
        }
        
        if(flowRateTimer > 0){//devide my 0 protection
            readFrequency = SAMPLE_FREQUENCY_HZ / flowRateTimer;
        }
        flowRateTimer = 0;
        numRises = 0;
        checkSpeed = false;
    }
}

float getSpeed(){
    calculateSpeed();
    return(readFrequency / (SPEED_CALIBRATION_CONSTANT));
}