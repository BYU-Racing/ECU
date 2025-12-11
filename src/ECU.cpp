#include "ECU.h"

constexpr int HORN_PIN = 19;
constexpr int BL_PIN = 13; // placeholder
constexpr int BTO_OFF_THRESHOLD = 120;
constexpr int BTO_ON_THRESHOLD = 300;
constexpr int INVERTER_PING_FREQUENCY = 100;

ECU::ECU() {
    throttle = Throttle();
    brake = Brake();
    tractiveActive = true; // for testing until we have a good way to read tractive 
    driveState = false;
    motorState = false;
    brakeOK = false;
    throttleOK = false;
    slipOK = true;
    BTOveride = false;
    throttle1UPDATE = false;
    torqueRequested = 0;
    throttleCode = 0;
    carIsGood = true;
    lastInverterPing = 0;
}

void ECU::setCAN(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> comsCANin, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> motorCANin) {
    comsCAN = comsCANin;
    motorCAN = motorCANin;
}


// initial diagnostics
void ECU::boot() {
    delay(150);
    carIsGood = runDiagnostics();
    pinMode(BL_PIN, OUTPUT);
}

bool ECU::runDiagnostics() {
    askForDiagnostics();  // starts diagnostic process
    return reportDiagnostics(); // reads diagnostics
}

void ECU::askForDiagnostics() {
    CAN_message_t msg{};
    msg.id = ReservedIDs::HealthCheckId; // diagnostic ID
    msg.len = 8;
    
    for (int i = 0; i < 8; i++) {
        msg.buf[i] = 0;
    }

    // write on the comsCAN
    comsCAN.write(msg);
}

bool ECU::reportDiagnostics() {
    // TODO: tweak once DCs are solidified
    timer = millis();
    data1Health = 0;
    data2Health = 0;
    data3Health = 0;

    while (millis() - timer <= 150) {
        CAN_message_t msg;
        if (msg.id == ReservedIDs::DCFId) {
            data1Health = msg.buf[0];
        }
        else if (msg.id == ReservedIDs::DCRId) {
            data2Health = msg.buf[0];
        }
        else if (msg.id == ReservedIDs::DCTId) {
            data3Health = msg.buf[0];
        }
    }

    return (data1Health >= 2 && data2Health >= 2 && data3Health >=2);
}


// start up and horn
void ECU::InitialStart() {
    Serial.println("INITIAL START ACHIEVED");
    digitalWrite(HORN_PIN, HIGH);
    delay(2000);  // delay for 2 seconds per rules
    digitalWrite(HORN_PIN, LOW);

    // send the driveState command for the dash
    CAN_message_t msg{};
    msg.id = ReservedIDs::DriveStateId;
    msg.len = 8;

    msg.buf[0] = 1;
    for (int i = 1; i < 8; i++) {
        msg.buf[i] = 0;
    }

    comsCAN.write(msg);

    // start the motor state logic
    driveState = true;
    sendMotorStartCommand();
}


// main loop
void ECU::run() {
    if (!driveState) {
        // TODO: start-fault feedback to driver?
        attemptStart();
    }

    // read COMS CAN into its own buffer, then copy into rmsg
    if (comsCAN.read(rmsg_coms)) {
        rmsg = rmsg_coms;
        route();
    }

    // read MOTOR CAN into its own buffer, then copy into rmsg
    if (motorCAN.read(rmsg_motor)) {
        rmsg = rmsg_motor;
        route();
    }

    if (lastInverterPing == 0 || millis() - lastInverterPing >= INVERTER_PING_FREQUENCY) {
        pingInverter();
    }

    if (!carIsGood) {
        shutdown();
    }
}


// keep the inverter alive
void ECU::pingInverter() {
    CAN_message_t ping{};
    ping.id = ReservedIDs::ControlCommandId;
    ping.len = 8;

    // zero torque, inverter enable bit set
    ping.buf[0] = 0;
    ping.buf[1] = 0;
    ping.buf[2] = 0;
    ping.buf[3] = 0;
    ping.buf[4] = 0;
    ping.buf[5] = 1;
    ping.buf[6] = 0;
    ping.buf[7] = 0;

    motorCAN.write(ping);
    lastInverterPing = millis();
}


// routes data
void ECU::route() {
    switch (rmsg.id) {
        // incoming sensor/command messages (data/coms CAN)
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

        // motor torque messages (from inverter) - forward to dashboard CAN
        case 172:  // torque and timer info
        case 176:  // high speed (torque and speed and DC bus)
            forwardToDashboard(rmsg);
            break;

        default:
            break;
    }
}


// update functions
void ECU::updateThrottle() {
    unpacker.reset(rmsg.buf);
    int32_t raw = unpacker.unpack<int32_t>();

    if (rmsg.id == ReservedIDs::Throttle1PositionId) {
        throttle.setThrottle1(raw);
        throttle1UPDATE = true;
        throttle1 = raw;  // store for calibration
    }
    else {
        throttle.setThrottle2(raw);
        throttle2UPDATE  = true;
        throttle2 = raw;  // store for calibration
    }

    // wait until both throttle channels have been updated
    if (!throttle1UPDATE || !throttle2UPDATE) {
        return;
    }

    torqueRequested = throttle.calculateTorque();
    throttleCode = throttle.checkError();
    throttleOK = (throttleCode == 0);
    // Serial.print("torque: ");
    // Serial.println(torqueRequested);

    if (!throttleOK) {
        throwError(throttleCode);
    }

    throttle1UPDATE = false;
    throttle2UPDATE = false;

    // send that command to the motor
    sendMotorCommand(torqueRequested);
}

// brake error handling
void ECU::updateBrake() {
    unpacker.reset(rmsg.buf);
    int32_t raw = unpacker.unpack<int32_t>();
    brake.updateValue(raw);

    brakeOK = (brake.getBrakeErrorState() != 2);

    // brake override patch
    if (!BTOveride) {
        if (!brakeOK) {
            throwError(FaultSourcesIDs::BrakeZeroId);
        }
    }
}

void ECU::updateSwitch() {
    prevStartSwitchState = startSwitchState;
    startSwitchState = (rmsg.buf[0] == 1);

    if (!startSwitchState && driveState) {
        // SHUTDOWN THE CAR!!!
        shutdown();
    }
}

// TODO: check that this function works for ECU mapping on the car
void ECU::updateDriveMode() {
    // driveMode variable is configured in ECU.h
    // For each mode, we write RPM limit and adjust max torque
    if (rmsg.buf[0] == 0 && driveMode == 0) {
        CAN_message_t msg{};
        msg.id  = 0x0C1;
        msg.len = 8;

        msg.buf[0] = 128;
        msg.buf[1] = 0;
        msg.buf[2] = 1; // write value
        msg.buf[3] = 0;
        msg.buf[4] = 255; // max RPM LSB (placeholder)
        msg.buf[5] = 255; // max RPM MSB (placeholder)
        msg.buf[6] = 0;
        msg.buf[7] = 0;

        motorCAN.write(msg);
        throttle.setMaxTorque(3100);
    } else if (rmsg.buf[0] == 1 && driveMode == 1) {
        CAN_message_t msg{};
        msg.id  = 0x0C1;
        msg.len = 8;

        msg.buf[0] = 128;
        msg.buf[1] = 0;
        msg.buf[2] = 1;
        msg.buf[3] = 0;
        msg.buf[4] = 255;
        msg.buf[5] = 255;
        msg.buf[6] = 0;
        msg.buf[7] = 0;

        motorCAN.write(msg);
        throttle.setMaxTorque(1550);
    } else if (rmsg.buf[0] == 2 && driveMode == 2) {
        CAN_message_t msg{};
        msg.id  = 0x0C1;
        msg.len = 8;

        msg.buf[0] = 128;
        msg.buf[1] = 0;
        msg.buf[2] = 1;
        msg.buf[3] = 0;
        msg.buf[4] = 255;
        msg.buf[5] = 255;
        msg.buf[6] = 0;
        msg.buf[7] = 0;

        motorCAN.write(msg);
        throttle.setMaxTorque(310);
    }
}


// action functions
void ECU::sendMotorStartCommand() {
    Serial.println("Motor start command sent");
    motorState = true;
}

void ECU::sendMotorStopCommand() {
    Serial.println("Motor stop command sent");
    motorState = false;
}

void ECU::sendMotorCommand(int torque) {
    // Ensure motorCommand is configured properly
    motorCommand.id  = ReservedIDs::ControlCommandId;
    motorCommand.len = 8;

    // If the motor has been commanded off but all conditions say it should be on
    if (!motorState && brakeOK && throttleOK && slipOK && driveState) {
        motorState = true;
    }

    // // manually overriding the brake
    // if (!motorState && throttleOK && slipOK && driveState) {
    //     motorState = true;
    // }

    if (driveState) {
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

    if (motorState && brakeOK && throttleOK && !BTOveride && driveState) {
        Serial.print("COMMANDED: ");
        Serial.println(torque);

        motorCommand.buf[0] = torque & 0xFF;
        motorCommand.buf[1] = (torque >> 8) & 0xFF;
        motorCommand.buf[2] = 0;
        motorCommand.buf[3] = 0;
        motorCommand.buf[4] = 0;
        motorCommand.buf[5] = 1; // RE-AFFIRMS THE INVERTER IS ACTIVE
        motorCommand.buf[6] = 0;
        motorCommand.buf[7] = 0;

        motorCAN.write(motorCommand);
        lastInverterPing = millis();
    } else if (motorState || !driveState) {
        // Sends a torque Message of 0 but keeps inverter enabled
        Serial.print("COMMANDED: ");
        Serial.println(0);

        motorCommand.buf[0] = 0;
        motorCommand.buf[1] = 0;
        motorCommand.buf[2] = 0;
        motorCommand.buf[3] = 0;
        motorCommand.buf[4] = 0;
        motorCommand.buf[5] = 1;
        motorCommand.buf[6] = 0;
        motorCommand.buf[7] = 0;

        motorCAN.write(motorCommand);
        lastInverterPing = millis();
    }
}

void ECU::shutdown() {
    driveState = false;
    BTOveride = false;
    Serial.println("SHUTDOWN");

    CAN_message_t msg{};
    msg.id = ReservedIDs::DriveStateId;
    msg.len = 8;

    for (int i = 0; i < 8; i++) {
        msg.buf[i] = 0;
    }

    comsCAN.write(msg);
    sendMotorStopCommand();
}

bool ECU::attemptStart() {
    // debug - you probably want real checks here later
    carIsGood = true;
    tractiveActive = true;

    if (brake.getBrakeActive() && !startFault && tractiveActive && carIsGood) {
        if (startSwitchState) {
            InitialStart();
            return true;
        }
        else if (prevStartSwitchState && !startSwitchState && startFault) {
            startFault = false;
        }
    }

    if (startSwitchState && !prevStartSwitchState) {
        // if we just flicked on the switch and did not satisfy the start conditions
        startFault = true;
        Serial.println("Start Faulted");
        // send message to driver screen about start fault
        throwError(FaultSourcesIDs::StartFaultId);
    }
    return false;
}

void ECU::checkBTOverride() {
    if (BTOveride && !brake.getBrakeActive() &&
        (torqueRequested <= BTO_OFF_THRESHOLD)) {
        BTOveride = false;
        Serial.println("BTO Set");
    }

    if (torqueRequested >= BTO_ON_THRESHOLD &&
        !BTOveride &&
        brake.getBrakeActive()) {
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
    CAN_message_t msg{};
    msg.id  = ReservedIDs::FaultId; // fault / diagnostic ID
    msg.len = 8;

    msg.buf[0] = code;
    for (int i = 1; i < 8; i++) {
        msg.buf[i] = 0;
    }

    // send the error code to the Dashboard
    comsCAN.write(msg);
}


// forward motor torque messages from motorCAN -> comsCAN
void ECU::forwardToDashboard(const CAN_message_t &msg) {
    CAN_message_t dashMsg = msg;   // copy the motor message directly
    comsCAN.write(dashMsg);        // send it to the dashboard
}