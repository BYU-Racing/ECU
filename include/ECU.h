#ifndef ECU_H
#define ECU_H

#include "FlexCAN_T4.h"
#include "Throttle.h"
#include "Brake.h"
#include "BufferPacker.h"
#include "Reserved.h"

//THE ECU MONITORS EVERYTHING ABOUT THE CAR AND DECIDES WHAT SHOULD BE DONE

class ECU {
    private:


        //COMS VARS
        FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> comsCAN;
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCAN;
        CAN_message_t rmsg;
        CAN_message_t motorCommand;

        //State Vars
        bool driveState = false;
        bool startFault = false;

        bool carIsGood = true;

        int driveMode = 0; //0 = Full beans, 1 = Endurance, 2 = SkidPad

        //MONITORING VARS
        BufferPacker<8> unpacker;

        //Diagnostics
        int data1Health = 0;
        int data2Health = 0;
        int data3Health = 0;
        unsigned int timer = 0;
        unsigned int lastInverterPing = 0;

        int wheelSpeed1Health = 0;
        int wheelSpeed2Health = 0;
        int wheelSpeed3Health = 0;
        int wheelSpeed4Health = 0;


        int dheelHealth = 0;
        
        //Start Switch
        bool startSwitchState = false;
        bool prevStartSwitchState = false;


        //Wheel Speed
        int fr_wheel_rpm;
        int fl_wheel_rpm;
        int rr_wheel_rpm;
        int rl_wheel_rpm;


        //GPS -> This may not have any real relevance
        float gps_lat;
        float gps_long;


        //Accelerometer
        float x_accel;
        float y_accel;
        float z_accel;

        float heading;

        float x_angle;
        float y_angle;
        float z_angle;

        //Steering wheel
        int steeringAngle;

        //Brake Sensor
        Brake brake;

        //Throttle Sensor
        int throttle1;
        int throttle2;
        int handoffCalVal1;
        int handoffCalVal2;

        Throttle throttle;

        bool throttle1UPDATE;
        bool throttle2UPDATE;

        int torqueRequested;

        int throttleCode;


        //CoolantLoop
        int coolantTemp1;
        int coolantTemp2;

        //Battery

        //Motor
        bool motorState;
        bool brakeOK;
        bool throttleOK;
        bool slipOK = true;

        bool BTOveride;

        //Tractive
        bool tractiveActive;


        //Car Motion
        float slipAngle;


    public:
        ECU();

        void setCAN(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> comsCANin, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCANin);

        //OVERALL CAR OPERATIONS
        void boot(); // -> initialBoot of car + diagnostics

        void run(); // -> WHILE LOOP RUNNING

        void InitialStart(); // -> HORN + COMMAND MOTOR

        void route(); // -> ROUTES DATA TO CORRECT SENSOR OP

        void shutdown();

        void pingInverter();




        //Individual Sensor Operations
        void updateThrottle();

        void updateBrake();

        void updateSwitch();

        void updateWheelSpeed();

        void updateAccelerometer();

        void updateCoolant();

        void updateSteering();

        void updateDriveMode();

        void updateGPS();


        //ACTION FUNCTIONS
        void sendMotorStartCommand();

        void sendMotorStopCommand();

        void sendMotorCommand(int torque); // -> Sends motor command needs to check for motor run conditions

        bool attemptStart(); // -> Attempts to start car if fails then goes into start fault

        void resetStartFault(); // -> Sees that all throttle / brake / switch are all zero before being able to attempt start again

        void calculateSlipAngle(); // -> Calculates slip angle for potential TC Control

        void checkBTOverride();


        bool runDiagnostics();

        void askForDiagnostics();

        bool reportDiagnostics();


        void calibrateThrottleMin();

        void calibrateThrottleMax();

        void throwError(int code);

        
};

#endif