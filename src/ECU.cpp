#include "ECU.h"

constexpr int HORN_PIN = 19; 
constexpr int BL_PIN = 13; //PLACEHOLDER

constexpr int BTO_OFF_THRESHOLD = 120;
constexpr int BTO_ON_THRESHOLD = 300;

constexpr int INVERTER_PING_FREQUENCY = 100;



ECU::ECU() {
    throttle = Throttle();
    brake = Brake();

    tractiveActive = true; //For testing until we come up with a good way to read tractive
}

void ECU::setCAN(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> comsCANin, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCANin) {
    comsCAN = comsCANin;
    motorCAN = motorCANin;
    
}

//Initial Diagnostics
void ECU::boot() {
    delay(150); //Makes sure ECU is last to be online so others can respond
    carIsGood = runDiagnostics();
    pinMode(BL_PIN, OUTPUT);
}

bool ECU::runDiagnostics() {
    askForDiagnostics(); //Starts Diagnostic Process

    return (reportDiagnostics()); // Reads Diagnostics
}

void ECU::askForDiagnostics() {
    //Just send the CAN message out for diagnositcs
    rmsg.id = ReservedIDs::HealthCheckId; // DIAGNOSTIC ID

    rmsg.len = 8;

    rmsg.buf[0] = 0;
    rmsg.buf[1] = 0;
    rmsg.buf[2] = 0;
    rmsg.buf[3] = 0;
    rmsg.buf[4] = 0;
    rmsg.buf[5] = 0;
    rmsg.buf[6] = 0;
    rmsg.buf[7] = 0;

    // Write on the comsCAN
    comsCAN.write(rmsg);
}

bool ECU::reportDiagnostics() {
    //TODO: This should get tweaked once DCs are solidified
    timer = millis();
    while(millis() - timer <= 150) {
        if(comsCAN.read(rmsg)) {
            if(rmsg.id == ReservedIDs::DCFId) {
                data1Health = rmsg.buf[0];
            }
            if(rmsg.id == ReservedIDs::DCRId) {
                data2Health = rmsg.buf[0];
            }
            if(rmsg.id == ReservedIDs::DCTId) {
                data3Health = rmsg.buf[0];
            }
        }
    }

    return (data1Health >= 2 && data2Health >= 2 && data3Health >= 2);
}

//START + HORN
void ECU::InitialStart() {
    Serial.println("INITIAL START ACHIEVED");
    digitalWrite(HORN_PIN, HIGH);
    delay(2000); // Delay for 2 seconds per rules
    digitalWrite(HORN_PIN,LOW);

    //Send the driveState command for the dash
    rmsg.id=ReservedIDs::DriveStateId;
    rmsg.buf[0]=1;
    rmsg.buf[1]=0;
    rmsg.buf[2]=0;
    rmsg.buf[3]=0;
    rmsg.buf[4]=0;
    rmsg.buf[5]=0;
    rmsg.buf[6]=0;
    rmsg.buf[7]=0;

    comsCAN.write(rmsg);
    //Start the motor
    driveState = true;

    sendMotorStartCommand();
}


//INGESTS MESSAGES AND ROUTES THEM (LOOP FUNCTION)
void ECU::run() {
    if(!driveState) {
        //TODO: SHOULD THIS SEND A START FAULT NOTICE TO THE DRIVER???
        attemptStart();

    }
    // read coms CAN line 
    if(comsCAN.read(rmsg)) {
        route();
    }
    // read motor CAN line 
    if(motorCAN.read(rmsg)) {
        route();
    }

    if (millis() - INVERTER_PING_FREQUENCY >= lastInverterPing || lastInverterPing == 0) {
        pingInverter();
    }

    if(!carIsGood) { // If something bad happened when running healthChecks
        shutdown();
    }
}

void ECU::pingInverter() {
    rmsg.len=8;
    rmsg.buf[0]=0;
    rmsg.buf[1]=0;
    rmsg.buf[2]=0;
    rmsg.buf[3]=0;
    rmsg.buf[4]=0;
    rmsg.buf[5]=0;
    rmsg.buf[6]=0;
    rmsg.buf[7]=0;
    rmsg.id=192;
    motorCAN.write(rmsg);
}

//ROUTES DATA (READS ID AND SENDS IT TO THE RIGHT FUNCTION)
void ECU::route() {

    switch (rmsg.id) {
        case ReservedIDs::Throttle1PositionId:
            updateThrottle();
            break;
        case ReservedIDs::Throttle2PositionId:
            updateThrottle();
            break;
        case ReservedIDs::BrakePressureId:
            updateBrake();
            break;
        case ReservedIDs::StartSwitchId:
            updateSwitch();
            break;
        case ReservedIDs::ThrottleMinId:
            calibrateThrottleMin();
            break;
        case ReservedIDs::ThrottleMaxId:
            calibrateThrottleMax();
            break;
        case ReservedIDs::DriveModeId:
            updateDriveMode();
            break;
        default:
            break;

    }
}




////////////////////////////////////////////
////////////UPDATE FUNCTIONS////////////////
////////////////////////////////////////////

void ECU::updateThrottle() {
    unpacker.reset(rmsg.buf);
    if(rmsg.id == ReservedIDs::Throttle1PositionId) {
        throttle.setThrottle1(unpacker.unpack<int32_t>());
        throttle1UPDATE = true;
    } else {
        throttle.setThrottle2(unpacker.unpack<int32_t>());
        throttle2UPDATE = true;
    }
    if(!throttle1UPDATE || !throttle2UPDATE) { // exits if both haven't been updated
        return;
    }

    torqueRequested = throttle.calculateTorque();

    throttleCode = throttle.checkError();

    throttleOK = (throttleCode == 0);
    
    if(!throttleOK) {
        throwError(throttleCode);
    }

    throttle1UPDATE = false;
    throttle2UPDATE = false;

    //Send that command to the motor
    sendMotorCommand(torqueRequested);
}

// TODO: Brake override path
void ECU::updateBrake() {
    unpacker.reset(rmsg.buf);
    brake.updateValue(unpacker.unpack<int32_t>());
    brakeOK = (brake.getBrakeErrorState() != 2); 

    // comment out to stop error handling with brake
    // if(!brakeOK) {
    //     throwError(FaultSourcesIDs::BrakeZeroId);
    // }
}

void ECU::updateSwitch() {
    prevStartSwitchState = startSwitchState;

    startSwitchState = (rmsg.buf[0] == 1);

    if(!startSwitchState && driveState) {
        //SHUTDOWN THE CAR!!!
        shutdown();
    }
}


//TODO: This dont really work for max RPM, needs further investigation into MCU control
void ECU::updateDriveMode() {
    if(rmsg.buf[0] == 0 && driveMode != 0) {
        if(driveMode == 2) { //RESET MAX RPM
            rmsg.id = 0x0C1;
            rmsg.buf[0] = 128;
            rmsg.buf[2] = 1; // 1 to write value

            rmsg.buf[4] = 255; // Write values for max RPM
            rmsg.buf[5] = 255;

            motorCAN.write(rmsg);
        }
        driveMode = 0;
        throttle.setMaxTorque(3100);
    }
    else if(rmsg.buf[0] == 1 && driveMode != 1) {
        if(driveMode == 2) { //RESET MAX RPM
            rmsg.id = 0x0C1;
            rmsg.buf[0] = 128;
            rmsg.buf[2] = 1; // 1 to write value

            rmsg.buf[4] = 255; // Write values for max RPM
            rmsg.buf[5] = 255;

            motorCAN.write(rmsg);
        }
        driveMode = 1;
        throttle.setMaxTorque(1500);
    }
    else if(rmsg.buf[0] == 2 && driveMode != 2) {
        //TODO: VERIFY THIS WORKING!!
        driveMode = 2;
        // Call the rpm limiter to the motor
        rmsg.id = 0x0C1;
        rmsg.buf[0] = 128;
        rmsg.buf[2] = 1; // 1 to write value

        rmsg.buf[4] = 255; // Write values for max RPM
        rmsg.buf[5] = 255;

        motorCAN.write(rmsg); 

        throttle.setMaxTorque(3100);
    }
}

/////////////////////////////////////////
////////////ACTION FUNCTIONS/////////////
/////////////////////////////////////////


//STOP/START BASE FUNCTIONS

void ECU::sendMotorStartCommand() {
    Serial.println("Motor start command sent");
    motorState = true;
    return;
}

void ECU::sendMotorStopCommand() {
    //Serial.println("Motor stop command sent");
    motorState = false;
    return;
}

void ECU::sendMotorCommand(int torque) {
    //Send the command to the motor
    if(!motorState && brakeOK && throttleOK && slipOK && driveState) { //If the motor has been commanded off but should be on
        motorState = true;
        //TODO: Determine if this needs to re start the inverter (it do not)
    }
    if(driveState) {
        checkBTOverride();
    }

    Serial.print("BRAKE: ");
    Serial.print(brakeOK);
    Serial.print(" TOK: ");
    Serial.print(throttleOK);
    Serial.print(" BTO: ");
    Serial.print(BTOveride);
    Serial.print(" DS: ");
    Serial.println(driveState);


    if(motorState && brakeOK && throttleOK && !BTOveride && driveState) {
        Serial.print("COMMANDED: ");
        Serial.println(torque);
        motorCommand.id = ReservedIDs::ControlCommandId;
        motorCommand.buf[0] = torque % 256;
        motorCommand.buf[1] = torque / 256;
        motorCommand.buf[2] = 0;
        motorCommand.buf[3] = 0;
        motorCommand.buf[4] = 0;
        motorCommand.buf[5] = 1; //RE AFFIRMS THE INVERTER IS ACTIVE
        motorCommand.buf[6] = 0;
        motorCommand.buf[7] = 0;
        motorCAN.write(motorCommand);
    }
    else if(motorState || !driveState) { //Sends a torque Message of 0
        Serial.print("COMMANDED: ");
        Serial.println(0);
        motorCommand.id = ReservedIDs::ControlCommandId;
        motorCommand.buf[0] = 0;
        motorCommand.buf[1] = 0;
        motorCommand.buf[2] = 0;
        motorCommand.buf[3] = 0;
        motorCommand.buf[4] = 0;
        motorCommand.buf[5] = 1; //RE AFFIRMS THE INVERTER IS ACTIVE
        motorCommand.buf[6] = 0;
        motorCommand.buf[7] = 0;
        motorCAN.write(motorCommand); 
    }

    return;
}


void ECU::shutdown() {
    driveState = false;
    BTOveride = false;
    Serial.println("SHUTDOWN");
    rmsg.id=ReservedIDs::DriveStateId;
    rmsg.buf[0]=0;
    rmsg.buf[1]=0;
    rmsg.buf[2]=0;
    rmsg.buf[3]=0;
    rmsg.buf[4]=0;
    rmsg.buf[5]=0;
    rmsg.buf[6]=0;
    rmsg.buf[7]=0;

    comsCAN.write(rmsg);
    sendMotorStopCommand();
}


bool ECU::attemptStart() {

    //DEBUG
    carIsGood = true;
    tractiveActive = true;

    if(brake.getBrakeActive() && !startFault && tractiveActive && carIsGood) {
        if(startSwitchState) {
            InitialStart();
            return true;
        }
    } else if(prevStartSwitchState && !startSwitchState && startFault) { // If we were in the fault position but are switching the switch off.
        startFault = false;
        //throwError(12); //Some error that the fault got cleared
    }
    if(startSwitchState && !prevStartSwitchState) { // If we just flicked on the switch and did not satisfy the start conditions
        startFault = true;
        Serial.println("Start Faulted");
            //SEND MESSAGE TO DRIVER SCREEN ABOUT START FAULT!!
        throwError(FaultSourcesIDs::StartFaultId);
    }
    return false;
}


void ECU::checkBTOverride() {

    if(BTOveride && !brake.getBrakeActive() && (torqueRequested <= BTO_OFF_THRESHOLD)) {
        BTOveride = false;
        Serial.println("BTO Set");
    }

    if(torqueRequested >= BTO_ON_THRESHOLD && !BTOveride && brake.getBrakeActive()) {
        BTOveride = true;
        Serial.println("BTO Released");
    }
}


void ECU::calibrateThrottleMin() {

    throttle.setCalibrationValueMin(throttle1, throttle2);
}

void ECU::calibrateThrottleMax() {

    throttle.setCalibrationValueMax(throttle1, throttle2);
}


void ECU::throwError(int code) {
    rmsg.id = ReservedIDs::FaultId; // DIAGNOSTIC ID

    rmsg.len = 8;

    rmsg.buf[0] = code;
    rmsg.buf[1] = 0;
    rmsg.buf[2] = 0;
    rmsg.buf[3] = 0;
    rmsg.buf[4] = 0;
    rmsg.buf[5] = 0;
    rmsg.buf[6] = 0;
    rmsg.buf[7] = 0;
    // Send the error code to the Dashboard
    comsCAN.write(rmsg);
}
