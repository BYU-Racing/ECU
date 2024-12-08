#ifndef THROTTLE_H
#define THROTTLE_H
#include <Arduino.h>
class Throttle {
    private:
        int throttle1 = 0;
        int throttle2 = 0;

        int maxTorque = 2200;

        int torque = 0;
        int rollingTorque = 0;

        int countMisMatch = 0;

        bool throttleError;
        bool throttleActive;

        bool throttle1UPDATE;
        bool throttle2UPDATE;

        int readIn1 = 0;
        int readIn2 = 0;
        int magiMemory[4];

        int minT1 = 8;
        int maxT1 = 1023;
        int minT2 = 8;
        int maxT2 = 1023;

    public:
        Throttle();

        int checkError();

        int calculateTorque();


        int getTorque();
        
        bool getStatus();

        bool getError();

        void setThrottle1(int input);
        void setThrottle2(int input);

        int consultMAGI(int input);

        bool getActive();

        void setCalibrationValueMin(int min1, int min2);
        void setCalibrationValueMax(int max1, int max2);
        
        void setMaxTorque(int torqueVal);
};

#endif