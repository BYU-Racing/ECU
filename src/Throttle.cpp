#include "Throttle.h"

constexpr int MIN_THROTTLE_OUTPUT = 0;
constexpr int MAX_THROTTLE_OUTPUT = 3100;

constexpr int MIN_THROTTLE_READ_POS = 4;
constexpr int MAX_THROTTLE_READ_POS = 1023;

constexpr int MIN_THROTTLE_READ_NEG = 4;
constexpr int MAX_THROTTLE_READ_NEG = 1023;
constexpr int THROTTLE_ERROR_TOL = 1600;
constexpr int THROTTLE_MAINTAIN_TOL = 20;
constexpr int THROTTLE_NOISE_REDUCTION_THRESHOLD = 60;


Throttle::Throttle() {
    magiMemory[0] = 0;
    magiMemory[1] = 0;
    magiMemory[2] = 0;
    magiMemory[3] = 0;
}



int Throttle::checkError() {
    if(abs(throttle1 - throttle2) < THROTTLE_ERROR_TOL) {
        countMisMatch = 0;
    } else {
        countMisMatch++;
    }

    if(countMisMatch >= THROTTLE_MAINTAIN_TOL) {
        return 1;
    }

    if(readIn1 == 0 || readIn2 == 0) {
        return 2;
    }
    
    return 0;
}

void Throttle::setThrottle1(int input) {
    readIn1 = input;

    Serial.print("T1: ");
    Serial.println(input);
    this->throttle1 = map(input, minT1, maxT1, MIN_THROTTLE_OUTPUT, maxTorque);
}

void Throttle::setThrottle2(int input) {
    readIn2 = input;
    Serial.print("T2: ");
    Serial.println(input);
    //Removing this so I can do the same throttle for testing on flatcar
    //this->throttle2 = map(-input, -maxT2, -minT2, MIN_THROTTLE_OUTPUT, maxTorque);
    this->throttle2 = map(input, minT1, maxT1, MIN_THROTTLE_OUTPUT, maxTorque);
}

int Throttle::calculateTorque() {
    torque = (throttle1 + throttle2) / 2;

    torque = consultMAGI(torque);

    if(torque < 0) {
        torque = 0;
    }

    return torque;
}


int Throttle::consultMAGI(int input) {

    // Add it to the memory
    rollingTorque = 0;

    this->magiMemory[3] = this->magiMemory[2];
    this->magiMemory[2] = this->magiMemory[1];
    this->magiMemory[1] = this->magiMemory[0];
    this->magiMemory[0] = torque;


    for(int i = 0; i < 4; i++) {
        if(this->magiMemory[i] == 0) {
            return 0;
        }
        rollingTorque += this->magiMemory[i];
    }

    return rollingTorque / 4;
}

bool Throttle::getActive() {
    return throttleActive;
}

void Throttle::setCalibrationValueMin(int min1, int min2) {
    maxT2 = min2;
    minT1 = min1;
}


void Throttle::setCalibrationValueMax(int max1, int max2) {
    minT2 = max2;
    maxT1 = max1;
}

void Throttle::setMaxTorque(int torqueVal) {
    maxTorque = torqueVal;
}