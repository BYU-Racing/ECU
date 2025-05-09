#include "Brake.h"



constexpr int BRAKE_THRESHOLD = 200;
constexpr int BL_PIN = 16; //PLACEHOLDER

// Strong error handling should help? driving dynamics might be worse without it tho

Brake::Brake() {
    brakeVal = 0;
}

bool Brake::getBrakeActive() {

    return (brakeVal >= BRAKE_THRESHOLD);
}

int Brake::getBrakeVal() {
    return brakeVal;
}

void Brake::updateValue(int data) {

    brakeVal = data;
    updateLight(); // call before updating brakeActive
    brakeActive = getBrakeActive();
    checkError();
}

void Brake::updateLight() {
    if(brakeActive && !getBrakeActive()) {
        digitalWrite(BL_PIN, LOW);
    } else if(!brakeActive && getBrakeActive()) {
        digitalWrite(BL_PIN, HIGH);
    }
}

bool Brake::checkError() {
    //Check if the pull-down resistor is active on the brake
    if(brakeVal <= 50) {
        
        if(errorState == 1 && (millis() - timeErrorStart) > 100) {
            errorState = 2; //Set critical error
            return true;
        } else if(errorState == 0) { //not in error currently
            errorState = 1; //Set in initial error
            timeErrorStart = millis();
            return false;
        }
        //Returns false if we are in an error but not critical
        return false;
    }
    //Moving out of critical errror state
    else if(errorState == 2) {
        timeErrorStart = 0;
        errorState = 0;
        return false;
    }
    //Moving out of the error state
    else if(errorState == 1) {
        errorState = 0;
        timeErrorStart = 0;
        return false;
    }
    return false; // Not sure what this value should be
}

int Brake::getBrakeErrorState() {
    return errorState;
}