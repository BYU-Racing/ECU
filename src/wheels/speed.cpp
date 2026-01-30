#include "speed.h"

static uint8_t lastRead = 0;
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
	// Only operates when checkSpeed flag is true
	if (!checkSpeed) { return; }

	// check to see if the current reading is high
	uint8_t currentRead = (analogRead(speedPin) > 300);

	if (currentRead == lastRead) {
		++flowRateTimer;
		return;
	} else {
		// Set the previous read value to the curretn
		lastRead = currentRead;

		if (currentRead == true) {
			++numRises;
		}

		// wait until we have enough data samples
		if (numRises < SAMPLE_COUNT) { 
			return;
		}

		if(flowRateTimer > 0) {  // protects against divide by 0
			readFrequency = SAMPLE_FREQUENCY_HZ / flowRateTimer;						
		}

		flowRateTimer = 0;
		numRises = 0;
		// reset the flag
		checkSpeed = false;
	}
}	

float getSpeed(){
    calculateSpeed();
    return(readFrequency / (SPEED_CALIBRATION_CONSTANT));
}
